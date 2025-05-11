import cv2
import numpy as np
import time
import os
import RPi.GPIO as GPIO
import tflite_runtime.interpreter as tflite
from picamera2 import Picamera2

# ========= GPIO MOTOR SETUP =========
GPIO.setmode(GPIO.BOARD)
IN1, IN2, IN3, IN4 = 11, 12, 15, 16
ENA, ENB = 32, 33
for pin in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(pin, GPIO.OUT)

pwmA = GPIO.PWM(ENA, 650)
pwmB = GPIO.PWM(ENB, 650)
pwmA.start(0)
pwmB.start(0)

def set_motor_speed(left, right):
    pwmA.ChangeDutyCycle(left)
    pwmB.ChangeDutyCycle(right)

def forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def backward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def left_pivot():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def right_pivot():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# ========= PID CONFIG =========
Kp, Ki, Kd = 0.45, 0.0006, 0.02
last_error, integral = 0, 0
last_direction = "left"

# ===== Teachable Machine Model =====
interpreter = tflite.Interpreter(model_path="/home/pi/Desktop/tensor/rightandquatercircle.tflite")
interpreter.allocate_tensors()
input_details  = interpreter.get_input_details()
output_details = interpreter.get_output_details()
with open("/home/pi/Desktop/tensor/rightandquatercircle.txt","r") as f:
    class_names = [ln.strip() for ln in f]
BACKGROUND_KEYWORDS = ['background','none','noise']
CONF_THRESHOLD = 0.55
DEBOUNCE_MS    = 1000
last_stop_time = 0

# ========= CAMERA SETUP =========
FRAME_WIDTH, FRAME_HEIGHT = 320, 240
picam2 = Picamera2()
picam2.preview_configuration.main.size   = (FRAME_WIDTH, FRAME_HEIGHT)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(2)

# ========= PATH TRACE MASK =========
path_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

# ========= ASSIGNED COLORS =========
assigned_colors = ['red', 'blue']

print("✅ Robot running — press q to quit.")

try:
    while True:
        frame = picam2.capture_array()
        now_ms = int(time.time()*1000)

        # --- 1. Symbol detection on upper ROI ---
        sym_roi = frame[0:int(FRAME_HEIGHT*0.6), :]
        inp = cv2.resize(sym_roi, (224, 224)).astype('float32') / 255.0
        inp = np.expand_dims(inp, axis=0)
        interpreter.set_tensor(input_details[0]['index'], inp)
        interpreter.invoke()
        probs = interpreter.get_tensor(output_details[0]['index'])[0]
        pred_idx = int(np.argmax(probs))
        pred_conf = float(probs[pred_idx])
        pred_label = class_names[pred_idx]

        label_lower = pred_label.lower()
        is_background = any(kw in label_lower for kw in BACKGROUND_KEYWORDS)

        # Draw ROI rectangle
        cv2.rectangle(frame, (0, 0), (FRAME_WIDTH, int(FRAME_HEIGHT*0.6)), (255, 0, 0), 2)

        # Draw predicted label
        display_text = f"{pred_label.upper()} ({pred_conf*100:.0f}%)"
        color = (0, 255, 0) if pred_conf >= CONF_THRESHOLD else (0, 0, 255)
        cv2.putText(frame, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Act on symbol if it's confident and not noise
        if (pred_conf >= CONF_THRESHOLD and not is_background 
                and (now_ms - last_stop_time) > DEBOUNCE_MS):
            stop()
            last_stop_time = now_ms
            print(f"🔍 Detected: {pred_label.upper()} ({pred_conf*100:.0f}%)")
            traced = cv2.addWeighted(frame, 1.0, path_mask, 1.0, 0)
            cv2.imshow("Line Path View", traced)
            cv2.waitKey(1)
            time.sleep(3)
            forward()
            time.sleep(0.8)

        # --- 2. Line following on lower ROI ---
        roi = frame[100:240, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        masks = {}
        if 'red' in assigned_colors:
            masks['red'] = cv2.inRange(hsv,(0,150,100),(10,255,255)) + \
                           cv2.inRange(hsv,(170,150,100),(180,255,255))
        if 'yellow' in assigned_colors:
            masks['yellow'] = cv2.inRange(hsv,(20,150,150),(35,255,255))
        if 'green' in assigned_colors:
            masks['green'] = cv2.inRange(hsv,(70,180,100),(90,255,180))
        if 'blue' in assigned_colors:
            masks['blue'] = cv2.inRange(hsv,(105,180,40),(120,255,150))
        masks['black'] = cv2.inRange(hsv,(0,0,0),(180,255,50))

        selected_mask = masks['black']; selected_color='black'
        for c in assigned_colors:
            if c in masks:
                cnts,_=cv2.findContours(masks[c],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                if cnts:
                    selected_mask=masks[c]; selected_color=c; break

        cnts,_=cv2.findContours(selected_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            largest=max(cnts,key=cv2.contourArea)
            M=cv2.moments(largest)
            if M['m00']:
                cx=int(M['m10']/M['m00'])
                frame_center=roi.shape[1]//2
                error=frame_center-cx
                integral+=error
                derivative=error-last_error
                output=Kp*error+Ki*integral+Kd*derivative
                last_direction='left' if error>0 else 'right'
                if abs(error)>90:
                    if error>0: left_pivot()
                    else:       right_pivot()
                    set_motor_speed(95,95); time.sleep(0.1); stop(); time.sleep(0.05)
                else:
                    base=33; L=max(0,min(100,base-output)); R=max(0,min(100,base+output))
                    forward(); set_motor_speed(L,R)
                last_error=error
        else:
            if last_direction=='left': left_pivot()
            else:                       right_pivot()
            set_motor_speed(95,95); time.sleep(0.13); stop(); time.sleep(0.05)

        traced_frame=cv2.addWeighted(frame,1.0,path_mask,1.0,0)
        cv2.imshow("Line Path View", traced_frame)
        if cv2.waitKey(1)&0xFF==ord('q'): break

except KeyboardInterrupt:
    pass
finally:
    stop(); pwmA.stop(); pwmB.stop(); GPIO.cleanup(); picam2.stop(); cv2.destroyAllWindows()
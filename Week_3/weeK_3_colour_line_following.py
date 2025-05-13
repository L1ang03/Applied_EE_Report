import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# ===== GPIO MOTOR SETUP =====
GPIO.setmode(GPIO.BOARD)
IN1, IN2, IN3, IN4 = 11, 12, 15, 16
ENA, ENB = 32, 33

for pin in [IN1, IN2, IN3, IN4, ENA, ENB]:
    GPIO.setup(pin, GPIO.OUT)

pwmA = GPIO.PWM(ENA, 500)
pwmB = GPIO.PWM(ENB, 500)
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

# ===== PID CONFIGURATION =====
Kp = 0.45
Ki = 0.0006
Kd = 0.02
last_error = 0
integral = 0
last_direction = "left"

# ===== CAMERA SETUP =====
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
picam2 = Picamera2()
picam2.preview_configuration.main.size = (FRAME_WIDTH, FRAME_HEIGHT)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()
time.sleep(2)

# ===== PATH TRACE MASK =====
path_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)

# ===== ASSIGNED COLORS (SET THIS BEFORE TEST) =====
assigned_colors = ['green']

try:
    while True:
        frame = picam2.capture_array()
        roi = frame[100:240, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ===== COLOR MASKS =====
        masks = {}

        if 'red' in assigned_colors:
            lower_red1 = np.array([0, 150, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 150, 100])
            upper_red2 = np.array([180, 255, 255])
            masks['red'] = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

        if 'yellow' in assigned_colors:
            lower_yellow = np.array([20, 150, 150])
            upper_yellow = np.array([35, 255, 255])
            masks['yellow'] = cv2.inRange(hsv, lower_yellow, upper_yellow)

        if 'green' in assigned_colors:
            lower_green = np.array([70, 180, 100])
            upper_green = np.array([90, 255, 180])
            masks['green'] = cv2.inRange(hsv, lower_green, upper_green)

        if 'blue' in assigned_colors:
            lower_blue = np.array([105, 180, 40])
            upper_blue = np.array([120, 255, 150])
            masks['blue'] = cv2.inRange(hsv, lower_blue, upper_blue)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        masks['black'] = cv2.inRange(hsv, lower_black, upper_black)

        # ===== PRIORITY SELECTION =====
        selected_mask = masks['black']
        selected_color = 'black'

        for color in assigned_colors:
            if color in masks:
                contours, _ = cv2.findContours(masks[color], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    selected_mask = masks[color]
                    selected_color = color
                    break

        # ===== CONTOUR AND PID CONTROL =====
        contours, _ = cv2.findContours(selected_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                global_cy = cy + 100
                frame_center = roi.shape[1] // 2
                error = frame_center - cx
                integral += error
                derivative = error - last_error
                output = Kp * error + Ki * integral + Kd * derivative
                last_direction = "left" if error > 0 else "right"

                if abs(error) > 90:
                    print(f"Sharp Turn [{selected_color.upper()}]")
                    if error > 0:
                        left_pivot()
                    else:
                        right_pivot()
                    set_motor_speed(95, 95)
                    time.sleep(0.1)
                    stop()
                    time.sleep(0.05)
                else:
                    base_speed = 40 if abs(error) < 60 else 35
                    left_speed = max(0, min(100, base_speed - output))
                    right_speed = max(0, min(100, base_speed + output))
                    print(f"Tracking [{selected_color.upper()}] | Error={error}, L={int(left_speed)} R={int(right_speed)}")
                    forward()
                    set_motor_speed(left_speed, right_speed)

                last_error = error
        else:
            print(f"Line Lost Keep turning {last_direction}")
            if last_direction == "left":
                left_pivot()
            else:
                right_pivot()
            set_motor_speed(95, 95)
            time.sleep(0.13)
            stop()
            time.sleep(0.05)

        traced_frame = cv2.addWeighted(frame, 1.0, path_mask, 1.0, 0)
        cv2.imshow("Line Path View", traced_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Quit key pressed.")
            break

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    stop()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()

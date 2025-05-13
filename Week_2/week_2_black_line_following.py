import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2


# Use BOARD numbering
GPIO.setmode(GPIO.BOARD)

# Motor driver pins (L298N)
IN1 = 11
IN2 = 12
IN3 = 15
IN4 = 16
ENA = 32
ENB = 33

# Set up pins as outputs
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Create PWM channels at 500Hz
pwmA = GPIO.PWM(ENA, 500)
pwmB = GPIO.PWM(ENB, 500)
pwmA.start(0)
pwmB.start(0)

def motor_stop():
    """Stop both motors."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def motor_speed(speed):
    """
    Set motor speed by changing duty cycle on both motors.
    :param speed: integer 0-100
    """
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)

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
	

def left():

    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
  
  
def right():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

   

# Initialize Camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (320, 240)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

time.sleep(2)

try:
    while True:
        # Capture frame
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  
        # Apply edge detection
        edges = cv2.Canny(gray, 50, 150)
        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
        if lines is not None:
            # Find the average x-coordinate of detected lines
            line_positions = [((x1 + x2) // 2) for x1, y1, x2, y2 in lines[:, 0]]
            avg_x = int(np.mean(line_positions))
            # Determine robot movement based on line position
            frame_center = frame.shape[1] // 2
            if avg_x < frame_center - 30:
                print("Turning Left")
                left()
                #time.sleep(0.3)
                motor_speed(98)
            elif avg_x > frame_center + 30:
                print("Turning Right")
                right()
                #time.sleep(0.3)
                motor_speed(98)	
            else:
                print("Moving Forward")
                forward()
                motor_speed(78)          
            #motor_speed(75)
            # Draw detected lines
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            print("No Line Detected")
            backward()
            motor_speed(48)
        # Display output
        cv2.imshow("Detected Lines", frame)
        cv2.imshow("Edges", edges)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Exiting...")
    stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
    picam2.stop()

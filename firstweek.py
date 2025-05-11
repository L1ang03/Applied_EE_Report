import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

IN1 = 11
IN2 = 12
IN3 = 15
IN4 = 16
ENA = 32  
ENB = 33  

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

pwmA = GPIO.PWM(ENA, 500)  
pwmB = GPIO.PWM(ENB, 500)  

pwmA.start(0)  
pwmB.start(0)  

def backward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def right():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)

def left():
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)





# Function to set motor speed (0 to 100)


def motor_speed(speed):
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)

# Example usage

try:
    motor_speed(75)
    left()
    time.sleep(1)
    stop()

finally:
    GPIO.cleanup()  # Clean up GPIOÂ pinsÂ whenÂ done
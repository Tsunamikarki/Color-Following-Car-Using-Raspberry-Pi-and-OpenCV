import cv2
import numpy as np
import RPi.GPIO as GPIO 
import time 

enRt = 19 
enLt = 13
rtFwd = 27
rtRev = 22
ltFwd = 23
ltRev = 24

# Servo 
servoPin = 18
duty=90

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(rtFwd, GPIO.OUT)
GPIO.setup(rtRev, GPIO.OUT)
GPIO.setup(ltFwd, GPIO.OUT)
GPIO.setup(ltRev, GPIO.OUT)
GPIO.setup(enRt, GPIO.OUT)
GPIO.setup(enLt, GPIO.OUT)
speedEnRt = GPIO.PWM(enRt, 1000)
speedEnLt = GPIO.PWM(enLt, 1000) 

# Servo Motor
GPIO.setup(servoPin, GPIO.OUT)
servo=GPIO.PWM(servoPin, 50)
servo.start(0)

def setServoAngle(angle):
    duty=angle / 18 + 2.5
    GPIO.output(servoPin, True)
    servo.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servoPin, False)
    servo.ChangeDutyCycle(0)
    print("Servo is working")

# Forward Motion
def forward():
    GPIO.output(rtFwd, GPIO.HIGH)
    GPIO.output(rtRev, GPIO.LOW)
    GPIO.output(ltFwd, GPIO.HIGH)
    GPIO.output(ltRev, GPIO.LOW)
    print("Moving forward!")

# Backward Motion
def reverse():
    GPIO.output(rtFwd, GPIO.LOW)
    GPIO.output(rtRev, GPIO.HIGH)
    GPIO.output(ltFwd, GPIO.HIGH)
    GPIO.output(ltRev, GPIO.LOW)

# Right Turn
def right():
	GPIO.output(rtFwd, GPIO.LOW)
	GPIO.output(rtRev, GPIO.HIGH)
	GPIO.output(ltFwd, GPIO.HIGH)
	GPIO.output(ltRev, GPIO.LOW)

# Left Turn
def left():
	GPIO.output(rtFwd, GPIO.HIGH)
	GPIO.output(rtRev, GPIO.LOW)
	GPIO.output(ltFwd, GPIO.LOW)
	GPIO.output(ltRev, GPIO.HIGH)    

#Stop Motors
def stop():
	GPIO.output(rtFwd, GPIO.LOW)
	GPIO.output(rtRev, GPIO.LOW)
	GPIO.output(ltFwd, GPIO.LOW)
	GPIO.output(ltRev, GPIO.LOW)

#start PWM
# speedEnRt.start(60) 
# speedEnLt.start(60)

video = cv2.VideoCapture(0)
while True: 
    ret, frame = video.read()
    frame = cv2.resize(frame,(640,480))
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    greenLower = np.array([90, 50, 50], np.int8)
    greenUpper = np.array([130, 255, 255], np.int8)
    greenMask = cv2.inRange(hsvImage, greenLower, greenUpper)
    
    greenFrame = cv2.bitwise_and(frame, frame, mask=greenMask)
    cv2.imshow('Video', frame)
    cv2.imshow('Green color detection', greenFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
# if __name__=="__main__":
    # speedEnRt.starst(60) 
    # speedEnLt.start(60)
    # forward()
    # time.sleep(2)
    # reverse()
    # time.sleep(2)
    # setServoAngle(0)
    # setServoAngle(90)
    # setServoAngle(180)
    # setServoAngle(90)
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
    speedEnRt.start(60) 
    speedEnLt.start(60)

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
    
    greenLower = np.array([35, 100, 100], np.uint8)
    greenUpper = np.array([85, 255, 255], np.uint8)
    greenMask = cv2.inRange(hsvImage, greenLower, greenUpper)
    
    
    cnts,_=cv2.findContours(greenMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
    detected = False
    for c in cnts:
        if cv2.contourArea(c)>600:
            x,y,w,h=cv2.boundingRect(c)
            cv2.rectangle(frame, (x,y),(x+w, y+h),(0,255,0),2)
            cv2.putText(frame,"DETECTED",(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),3)
            
            detected = True
            
            
    if detected:
        forward()
    else:
        stop()
        print("No color detected. Stop the motors.")
        
    greenResult=cv2.bitwise_and(frame,frame,mask=greenMask)        
    cv2.imshow("Original Frame", frame)
    # cv2.imshow("Original Frame", frame)
    # cv2.imshow("Result", greenResult)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
# if __name__=="__main__":
#    speedEnRt.starst(60) 
#    speedEnLt.start(60)
#    forward()
#    time.sleep(2)
    # reverse()
    # time.sleep(2)
    # setServoAngle(0)
    # setServoAngle(90)
    # setServoAngle(180)
    # setServoAngle(90)
import cv2 
import numpy as np

# Servo 
servoPin = 18
duty=90

def setServoAngle(angle):
    duty=angle / 18 + 2.5
    GPIO.output(servoPin, True)
    servo.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servoPin, False)
    servo.ChangeDutyCycle(0)
    print("Servo is working")
    
setServoAngle(0)
video=cv2.VideoCapture(0)
xMedium = 0
while True:
    ret, frame = video.read()
    frame = cv2.resize(frame,(640,480))
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # For green color
    greenLower = np.array([35, 100, 100], np.uint8)
    greenUpper = np.array([85, 255, 255], np.uint8)
    greenMask = cv2.inRange(hsvFrame, greenLower, greenUpper)
    
    contours,_= cv2.findContours(greenMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    for cnt in contours:
        (x,y,w,h) = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x,y),(x+w, y+h),(0,255,0),2)
        cv2.putText(frame,"DETECTED",(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),3)
        xMedium = int((x + x + w)/2)
        
        break
        
    #xMedium = int(x + w + x)/2
    
    cv2.line(frame, (xMedium, 0), (xMedium,480), (0, 255, 0), 2)
    cv2.imshow("Original Frame", frame)
    #cv2.imshow("Green", greenMask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
import cv2
import numpy as np

video = cv2.VideoCapture(0)

while True:
    ret, frame = video.read()
    frame = cv2.resize(frame,(1280,640))
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    greenLower = np.array([35, 100, 100], np.uint8)
    greenUpper = np.array([85, 255, 255], np.uint8)
    greenMask = cv2.inRange(hsvImage, greenLower, greenUpper)
    
    
    cnts,_=cv2.findContours(greenMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    
    for c in cnts:
        if cv2.contourArea(c)>600:
            x,y,w,h=cv2.boundingRect(c)
            cv2.rectangle(frame, (x,y),(x+w, y+h),(0,255,0),2)
            cv2.putText(frame,"DETECTED",(10,60),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),3)
    greenResult=cv2.bitwise_and(frame,frame,mask=greenMask)        
    cv2.imshow("Original Frame", frame)
    # cv2.imshow("Original Frame", frame)
    # cv2.imshow("Result", greenResult)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
import cv2
import numpy as np

video = cv2.VideoCapture(0)

while True:
    ret, frame = video.read()
    frame = cv2.resize(frame,(640,480))
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    greenLower = np.array([36, 25, 25], np.int8)
    greenUpper = np.array([86, 255, 255], np.int8)
    greenMask = cv2.inRange(hsvImage, greenLower, greenUpper)
    _,mask1=cv2.threshold(greenMask,254,255,cv2.THRESH_BINARY)
    cnts,_=cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for c in cnts:
        x=600
        if cv2.contourArea(c)>x:
            x,y,w,h=cv2.boundingRect(c)
            cv2.rectangle(frame(x,y),(x+w, y+h),(0,255,0),2)
            cv2.putText(frame,("DETECTED"),(10,60),cv2.FONT_HERSHEY_SIMPLEX,0,6,(0,0,255),2)
    greenResult=cv2.bitwise_and(frame,frame,mask=greenMask)        
    cv2.imshow("FRAME", frame)
    cv2.imshow("Result", greenResult)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
video.release()
cv2.destroyAllWindows()
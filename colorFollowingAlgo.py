import numpy as np
import cv2 
import RPi.GPIO as GPIO 
import time 

video = cv2.VideoCapture(0)

while True: 
    ret, frame = video.read()
    

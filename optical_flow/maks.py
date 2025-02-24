import cv2
import numpy as np


# Masking is just You take a specific area in image nd show/work with it

video = cv2.VideoCapture(0)

while True:
    ret , frame = video.read()
    black = np.zeros(frame.shape[:2], np.uint8)

    mask = cv2.circle(black , (100 , 100) , 60 , (255,255,255) , -1)

    frame = cv2.bitwise_and(frame , frame , mask=mask)

 
    cv2.imshow("haha",frame)
    if cv2.waitKey(1)==ord("q"):
        break

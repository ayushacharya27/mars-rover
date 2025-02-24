import cv2
import numpy as np
import matplotlib.pyplot as plt


vid = cv2.VideoCapture(0)

kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]]) # For Sharpening images



while True:
    ret , frame = vid.read()
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    frame = cv2.filter2D(frame , -1 , kernel)

    corner = cv2.goodFeaturesToTrack(frame , maxCorners=100 , qualityLevel=0.3 , minDistance= 7) # Returns an Array
    corners = np.int8(corner) # Converting to int

    for i in corners:
        x,y = i.ravel() # ravel convert 2d array into 1d
        cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)

    cv2.imshow("haha",frame)
    if cv2.waitKey(1)==ord("q"):
        break

    

    print(corners)


import cv2
import numpy as np

# For Sharpening of Images
kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])

video = cv2.VideoCapture(2)

ret , cap1 = video.read()
frame = cv2.cvtColor(cap1,cv2.COLOR_BGR2GRAY)

# Sharpening using Kernels
frame = cv2.filter2D(frame , -1 , kernel)

# Tracking Features
p0 = cv2.goodFeaturesToTrack(frame , maxCorners=100 , qualityLevel=0.3 , minDistance= 7)

mask = np.zeros_like(cap1)


while True:
    ret , cap = video.read()
    cap_gray = cv2.cvtColor(cap,cv2.COLOR_BGR2GRAY)

    cap_gray = cv2.filter2D(cap_gray , -1 , kernel)

    # p1 is the new points , st is status if points captured are not  and err is the error of calculating the points
    p1 , st , err = cv2.calcOpticalFlowPyrLK(frame ,cap_gray , p0 ,None )

    if p1 is not None:
        good_new = p1[st==1]
        good_old = p0[st==1]

    for i , j in zip(good_new , good_old):
        a , b = i.ravel()
   
        c , d = j.ravel()

        cv2.line(mask ,(int(a),int(b)),(int(c),int(d)), (255,255,255),1) # Without mask it just renews itself in every frame change
            # so now we use mask and its constant , it doesn't change
            
    output = cv2.add(cap , mask)
    cv2.imshow("haha",output)
    if(cv2.waitKey(1)==ord('q')):
        break
            
    frame = cap_gray.copy()

    # assume if there is no change so it might just do something wrong so we have to make a condition and reshape it also

    if(len(good_new)>0):
        p0 = good_new.reshape(-1 , 1 , 2) # Why Reshaping? Answer : see line 30 You are taking only specific portions so to make it original shape what the good features , retrun we reshape

    

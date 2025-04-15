import cv2
import numpy as np

sift = cv2.SIFT_create()

# Sharpening Kernel
kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])

# Load images
img1_1 = cv2.imread("img1.jpg")
img2_1 = cv2.imread("img2.jpg")

 
img1 = cv2.cvtColor(img1_1, cv2.COLOR_BGR2GRAY)
img2 = cv2.cvtColor(img2_1, cv2.COLOR_BGR2GRAY)
 
img1 = cv2.filter2D(img1, -1, kernel)
img2 = cv2.filter2D(img2, -1, kernel)

 
#p0 = cv2.goodFeaturesToTrack(img1, maxCorners=500, qualityLevel=0.2, minDistance=5)
#kp1, des1 = sift.detectAndCompute(img1, None)
#p0 = np.array([kp.pt for kp in kp1], dtype=np.float32).reshape(-1, 1, 2)

orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(img1, None)

p0 = np.array([kp.pt for kp in kp1], dtype=np.float32).reshape(-1, 1, 2)
 
 
p1, st, err = cv2.calcOpticalFlowPyrLK(img1, img2, p0, None)

# Filter only matched points (st == 1 means successfully tracked)
good_new = p1[st.flatten() == 1]
good_old = p0[st.flatten() == 1]

 
for new, old in zip(good_new, good_old):
    x_new, y_new = new.ravel()
    x_old, y_old = old.ravel()
    cv2.circle(img2_1, (int(x_new), int(y_new)), 3, (0, 255, 0), -1)   
    cv2.circle(img1_1, (int(x_old), int(y_old)), 3, (0, 0, 255), -1) 
    cv2.line(img2_1, (int(x_new), int(y_new)), (int(x_old), int(y_old)), (255, 0, 0), 1)  

 
while True:
    cv2.imshow("Image 1", img1_1)
    cv2.imshow("Image 2", img2_1)
    if cv2.waitKey(1) == ord("q"):
        break

 

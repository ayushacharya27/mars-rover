#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

# For Sharpening of Images
kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])

first_frame_flag = False  # For taking feature points for the first frame only
mask = None
first_frame = None
p0 = None

bridge = CvBridge()

def image_callback(ros_image):
    global mask, first_frame_flag, first_frame, p0

    # Convert ROS Image to OpenCV format
    frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Sharpening using Kernels
    sharp_image = cv2.filter2D(gray_image, -1, kernel)

    if not first_frame_flag:
        # Detect Good Features to Track (Shi-Tomasi Corners)
        p0 = cv2.goodFeaturesToTrack(sharp_image, maxCorners=100, qualityLevel=0.3, minDistance=7)

        # Initialize mask for drawing optical flow
        mask = np.zeros_like(frame)
        first_frame = sharp_image.copy()
        first_frame_flag = True
        return

    # Compute Optical Flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(first_frame, sharp_image, p0, None)

    if p1 is not None:
        good_new = p1[st == 1]  # Extracting only useful points (status == 1)
        good_old = p0[st == 1]

        # Draw Optical Flow Lines
        for i, j in zip(good_new, good_old):
            c, d = map(int, i.ravel())  # New position
            a, b = map(int, j.ravel())  # Old position
            cv2.line(mask, (a, b), (c, d), (255, 0, 0), 1)

        output = cv2.add(frame, mask)

        # Select 3 feature points for transformation estimation
        if len(good_new) >= 3:
            pts0 = np.float32(good_old[:3]).flatten()
            pts1 = np.float32(good_new[:3]).flatten()

            # Publish feature points
            msg_pts = Float32MultiArray()
            msg_pts.data = np.concatenate((pts0, pts1)).tolist()
            pub.publish(msg_pts)

            # 🔹 SHOW SELECTED POINTS ON IMAGE
            for (x, y) in pts0.reshape(-1, 2):
                cv2.circle(output, (int(x), int(y)), 5, (0, 255, 0), -1)  # Green - Old Points
            for (x, y) in pts1.reshape(-1, 2):
                cv2.circle(output, (int(x), int(y)), 5, (0, 0, 255), -1)  # Red - New Points

            cv2.imshow("Tracked Feature Points (Optical Flow)", output)
            cv2.waitKey(1)

            # Update previous frame
            first_frame = sharp_image.copy()
            p0 = good_new.reshape(-1, 1, 2)

if __name__ == '__main__':
    rospy.init_node("visual_odometry_node", anonymous=True)
    pub = rospy.Publisher("optical_flow_points", Float32MultiArray, queue_size=10)
    rospy.Subscriber("camera_image_raw", Image, image_callback)
    
    rospy.spin()
    cv2.destroyAllWindows()

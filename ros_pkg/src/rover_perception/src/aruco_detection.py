#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2.aruco import ArucoDetector



def detect_aruco(img_msg):
    # Making a Bridge b/w the Ros Image and ROS
    bridge = CvBridge()
    cv_image_og = bridge.imgmsg_to_cv2(img_msg)
    cv_image = cv_image_og.copy()  # Ensure the array is writable
 


    # Use DICT_4X4_100 (supports IDs 0-99)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = ArucoDetector(aruco_dict, parameters)

    height , width , _ = cv_image.shape
    frame_center = width/2

    # Detecting Corners of Aruco
    corners, ids, _ = detector.detectMarkers(cv_image)

    for i in range(len(corners)):
        corner = corners[i][0]
        x_center = int((corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4)
    
    if ids is not None:
        print("Detected IDs:", ids.flatten())
        print(frame_center- x_center)  # Should show IDs up to 999
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        cv2.imshow("ArUco Detection", cv_image)
        


if __name__ == '__main__':
    bridge = CvBridge()
    rospy.init_node('camera_sub', anonymous=True)
    sub_image = rospy.Subscriber("camera_image_raw", Image, detect_aruco)
    rospy.spin()
         


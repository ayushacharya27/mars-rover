#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray , Float32
from sensor_msgs.msg import Image
import cv2
from cv2.aruco import ArucoDetector
from cv_bridge import CvBridge, CvBridgeError

def center_and_distance(img_msg):
    # Publisher publishes only if it subscribes to camera node
    
    pub = rospy.Publisher("aruco_center_distance" , Float32 , queue_size=10)
    rate = rospy.Rate(10)

    # Making a Bridge b/w the Ros Image and ROS
    bridge = CvBridge()
    cv_image_og = bridge.imgmsg_to_cv2(img_msg)
    cv_image = cv_image_og.copy()  # Ensure the array is writable

    height,width,_ = cv_image.shape
    frame_center = width/2

    # Use DICT_4X4_100 (supports IDs 0-999)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = ArucoDetector(aruco_dict, parameters)

     # Detecting Corners of Aruco
    corners, ids, _ = detector.detectMarkers(cv_image)
    for i in range(len(corners)):
        corner = corners[i][0]
        x_center = int((corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]) / 4)

    if ids is not None:
        pub.publish(frame_center - x_center)
        rate.sleep()

    
if __name__ == '__main__':
    rospy.init_node("aruco_pid_node" , anonymous= True )
    sub_image = rospy.Subscriber("camera_image_raw", Image, center_and_distance)
    rospy.spin()





 



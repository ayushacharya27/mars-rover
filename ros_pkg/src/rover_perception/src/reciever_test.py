#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()

def image_callback(ros_image):
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        cv2.imshow("frame", frame)

        if cv2.waitKey(1) == ord('q'):
            rospy.signal_shutdown("User Pressed Q")
    except CvBridgeError as e:
        rospy.logerr(f"CV Bridge error: {e}")

if __name__ == '__main__':
    rospy.init_node("reciever_node", anonymous=True)
    rospy.Subscriber("camera_image_optical", Image, image_callback)
    
    rospy.spin()
    cv2.destroyAllWindows()  # Ensure OpenCV windows close properly

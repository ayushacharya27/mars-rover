#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray


# For Sharpening of Images
kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])

first_frame_flag = False # For taking features points for the first frame only
mask = None
first_frame = None
p0 = None

#rate = rospy.Rate(10) We cant define it before intitialising the node so it's dogshit

bridge = CvBridge()

def image_callback(ros_image):
    global mask , first_frame_flag , first_frame , p0

    #rospy.init_node("optical_flow_node" , anonymous= True)
    


    # Loop for the first Frame Calculation and Taking points
    if not first_frame_flag:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        gray_image = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        # Sharpening using Kernels
        sharp_image = cv2.filter2D(gray_image , -1 , kernel)

        # Calculating Feautures and their co-ordinates for the given Frame
        p0 = cv2.goodFeaturesToTrack(sharp_image , maxCorners=100 , qualityLevel=0.3 , minDistance= 7)
        #rospy.loginfo(p0)

        #mask = np.zeros_like(sharp_image)  '''Its Wrong tho since we need to mask the original bgr image'''
        #mask = np.zeros_like(frame)
        first_frame = sharp_image
        first_frame_flag = True

    # Loop for the second and ongoing frames
    if first_frame_flag:
        frame = bridge.imgmsg_to_cv2(ros_image,"bgr8")

        # PreProcessing the Images
        gray_image = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        sharp_image = cv2.filter2D(gray_image , -1 , kernel)

        # Calculation of Optical Flow Points
        p1 , st , err = cv2.calcOpticalFlowPyrLK(first_frame , sharp_image , p0 , None)

        if p1 is not None :
            good_new  = p1[st==1] # extracting only useful points i.e. status == 1 
            good_old = p0[st==1] 

        for i, j in zip(good_new, good_old):
            c, d = map(int, i.ravel())  # New position
            a, b = map(int, j.ravel())  # Old position
            cv2.line(mask, (a, b), (c, d), (255, 0, 0), 1)

        output = cv2.add(frame , mask)
        pts0 = np.float32(good_old)  # selecting any 3 points and will calc the tx , ty ... from Transformation_Matrix
        pts1 = np.float32(good_new) 

        displacements = pts1 - pts0
        tx, ty = np.mean(displacements, axis=0)

        '''if len(good_new) >= 3:
            # Sort points based on stability (smallest movement between frames)
            distances = np.linalg.norm(good_old - good_new, axis=1)
            sorted_indices = np.argsort(distances)[:3]  # Select top 3 most stable points
    
            pts0 = np.float32([good_old[i] for i in sorted_indices]).flatten()
            pts1 = np.float32([good_new[i] for i in sorted_indices]).flatten()
        else:
            rospy.logwarn("Not enough stable points detected, skipping frame update")
        return'''


        msg_pts = Float32MultiArray() # We are using this Message
        #msg_pts.data = np.concatenate((pts0.flatten(), pts1.flatten())).tolist() # combining and making it a one array
        msg_pts.data = [tx , ty]
        print(tx,ty)
        pub.publish(msg_pts)

        first_frame = sharp_image.copy()
        p0 = good_new.reshape(-1,1,2)

        


        #rospy.loginfo(combined_points) Just for Debugging
        #print(combined_points)

        #output_image = bridge.cv2_to_imgmsg(output , "bgr8")
        #pub.publish(output_image)
        #rate.sleep() no need since callback sutomatically responds to the publisher rate

        



                                







 
if __name__ == '__main__':
    rospy.init_node("visual_odometry_node", anonymous=True)
    pub = rospy.Publisher("optical_flow_points" , Float32MultiArray , queue_size=10)
    rospy.Subscriber("camera_image_raw", Image, image_callback)
    
    rospy.spin()
    cv2.destroyAllWindows()  # Ensure OpenCV windows close properly

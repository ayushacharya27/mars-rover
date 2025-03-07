#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

x = 0.0 
y = 0.0

z = 0

tx1 = 0.0
ty1 = 0.0


def flow_points_callback(msg):
    global x , y , tx1 , ty1  , z
    data = np.array(msg.data , dtype=np.float32)
    #print(data)

    if len(data) != 48 or len(data)==0 :
        rospy.logerr(f"Must be 24 Points but given:{len(data)}")
        return
    

    half = int(len(data)/2)
    
    pts0 = data[:half].reshape(12,2) # Reshaping into (3,2) array
    pts1 = data[half:].reshape(12,2)

    #transformation_matrix = cv2.getAffineTransform(pts0, pts1) # Giving it More Points
    # It only works for 3 points

    # It Takes more than 3 Points
    transformation_matrix, _ = cv2.estimateAffinePartial2D(pts0, pts1, method=cv2.RANSAC)

    tx, ty = transformation_matrix[:, 2]


    if(z==5):
        x += tx1
        y += ty1

        z = 0
    
    else:
        z+=1
        tx1 += tx/z
        ty1 += ty/z

    



    



 
    print(x , y)
    

    # Next Steps: Apply Kalmann Filter and more noise reduction Techniques














rospy.init_node("trajectory_node" , anonymous=True)
rospy.Subscriber("optical_flow_points" , Float32MultiArray , flow_points_callback)

rospy.spin()

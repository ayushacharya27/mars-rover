#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

x = 0.0 
y = 0.0

tx1 = 0.0
ty1 = 0.0


def flow_points_callback(msg):
    global x , y , tx1 , ty1
    data = np.array(msg.data , dtype=np.float32)
    #print(data)

    if len(data) != 24 or len(data)==0 :
        rospy.logerr("Must be 24 Points")
        return
    

    half = int(len(data)/2)
    
    pts0 = data[:half].reshape(6,2) # Reshaping into (3,2) array
    pts1 = data[half:].reshape(6,2)

    #transformation_matrix = cv2.getAffineTransform(pts0, pts1) # Giving it More Points
    # It only works for 3 points

    # It Takes more than 3 Points
    transformation_matrix, _ = cv2.estimateAffinePartial2D(pts0, pts1, method=cv2.RANSAC)

    tx, ty = transformation_matrix[:, 2]

    tx1 +=tx
    ty1 +=ty

    #print(tx1 , ty1)
 




    if (tx1 <= -8 and ty1 <= -8) :
        x+=1
        y+=1

        tx1=0
        ty1=0
    elif  (tx1 >=8 or ty1 >=8):
        x-=1
        y-=1




    # Making an Threshold
    #if tx >1  and ty > 1 or tx <-1 and ty < -1: # This wont work bro because bot moves at constant speed so assume at particular instant the difference might be very low not be 5
        #x-=tx
        #y-=ty

    print(x , y)
    

    # Next Steps: Apply Kalmann Filter and more noise reduction Techniques














rospy.init_node("trajectory_node" , anonymous=True)
rospy.Subscriber("optical_flow_points" , Float32MultiArray , flow_points_callback)

rospy.spin()

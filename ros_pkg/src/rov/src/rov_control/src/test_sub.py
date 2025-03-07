#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def callback(msg):
    rospy.loginfo(f"Received cmd_vel: Linear={msg.linear.x}, Angular={msg.angular.z}")

def listener():
    rospy.init_node("rover_controller", anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()

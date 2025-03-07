#!/usr/bin/env python3
import rospy
import keyboard
from std_msgs.msg import Char

key = None

def keys_publisher():
    global key
    rospy.init_node("keyboard_publisher_node" , anonymous=True)
    pub = rospy.Publisher("key_publisher" , Char , queue_size=10)



    if (keyboard.is_pressed('w')):
        while(keyboard.is_pressed('w')):
            key = 'w'
            pub.publish(key)

    pub.publish('n')



keys_publisher()







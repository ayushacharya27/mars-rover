#!/usr/bin/env python3
import rospy
import keyboard
from std_msgs.msg import String

#key = None Global Value not required

def keys_publisher():
    global key
    rospy.init_node("keyboard_publisher_node" , anonymous=True)
    pub = rospy.Publisher("key_publisher" , String , queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        key = None
        if (keyboard.is_pressed('w')):
            key = 'w'
        elif (keyboard.is_pressed('a')):
            key = 'a'
        elif (keyboard.is_pressed('s')):
            key = 'a'
        elif (keyboard.is_pressed('d')):
            key = 'a'
        else:
            key = 'n'
             

        pub.publish(key)
        rate.sleep()


if __name__ == "__main__":
    keys_publisher()







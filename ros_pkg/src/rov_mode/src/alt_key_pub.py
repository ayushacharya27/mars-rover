#!/usr/bin/env python3
import rospy
from pynput import keyboard
from std_msgs.msg import String

key_pressed = None

def on_pressed(key):
    global key_pressed
    key_pressed = key.char

def on_release(key):
    global key_pressed
    key_pressed = 'n'
    




def keys_publisher():
    global key_pressed
    
    rospy.init_node("keyboard_publisher_node" , anonymous=True)
    pub = rospy.Publisher("key_publisher" , String , queue_size=10)


    rate = rospy.Rate(10)

    listener = keyboard.Listener(on_press=on_pressed, on_release=on_release)
    listener.start()
    while not rospy.is_shutdown():
        pub.publish(key_pressed)
        rospy.loginfo(key_pressed)
        rate.sleep()


if __name__ == "__main__":
    keys_publisher()






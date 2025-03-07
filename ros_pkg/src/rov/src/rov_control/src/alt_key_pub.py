#!/usr/bin/env python3
import rospy
from pynput import keyboard
from std_msgs.msg import String

# Store currently pressed keys
pressed_keys = set()

def on_press(key):
    try:
        if key.char in ['w', 'a', 's', 'd']:  # Capture only movement keys
            pressed_keys.add(key.char)
    except AttributeError:
        pass  # Ignore special keys

def on_release(key):
    try:
        if key.char in pressed_keys:
            pressed_keys.remove(key.char)
    except AttributeError:
        pass

def keys_publisher():
    rospy.init_node("keyboard_publisher_node", anonymous=True)
    pub = rospy.Publisher("key_publisher", String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        if pressed_keys:
            key_to_publish = list(pressed_keys)[-1]  # Get the last pressed key
        else:
            key_to_publish = 'n'  # Default when no keys are pressed

        pub.publish(key_to_publish)
        rospy.loginfo(f"Published: {key_to_publish}")
        rate.sleep()

if __name__ == "__main__":
    keys_publisher()

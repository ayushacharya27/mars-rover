#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32MultiArray

# Initialize serial port
SERIAL_PORT = "/dev/ttyACM0"  # Change this as per your setup
BAUD_RATE = 9600   

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    rospy.loginfo(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
except serial.SerialException:
    rospy.logerr(f"Could not open serial port {SERIAL_PORT}")
    exit(1)

def mode_callback(data):
     
    if len(data.data) != 2:
        rospy.logerr("Received malformed data")
        return
    
    arm_flag = data.data[0]  # 1 for armed, 0 for disarmed
    mode = data.data[1]  # ASCII value of 'm', 'o', 'p', or 'u'

    serial_msg = f"{arm_flag},{chr(mode)}\n"  # Convert ASCII to char
    ser.write(serial_msg.encode())  # Send via serial
    
    rospy.loginfo(f"Sent to Serial: {serial_msg.strip()}")

def subscriber():
 
    rospy.init_node("mode_serial_subscriber", anonymous=True)
    rospy.Subscriber("mode_publisher", Int32MultiArray, mode_callback)
    rospy.spin()  # Keep node running

if __name__ == "__main__":
    subscriber()

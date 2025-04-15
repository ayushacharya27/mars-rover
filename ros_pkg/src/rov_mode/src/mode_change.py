#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

''' Modes Of Operation:
m - Manual Mode
o - Arming the Vehicle
p - Disarming the Vehicle 
u - Autonomous Mode
'''

# Flags
arm_flag = 0  # 0 = Disarmed, 1 = Armed
mode = ord('m')  # Default mode is 'm' (Manual)
mode_data = [arm_flag, mode]  # [Arming Flag, Mode]

def change_commands():
    global arm_flag, mode, mode_data
    rospy.init_node("mode_publisher_node", anonymous=True)
    pub = rospy.Publisher("mode_publisher", Int32MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        user_input = input("\nEnter command (o: Arm, p: Disarm, m: Manual, u: Autonomous): ").strip()

        if len(user_input) == 1:
            if user_input == 'o' and arm_flag == 0:
                arm_flag = 1
                mode = ord('m')
                rospy.logwarn("VEHICLE ARMED")
            
            elif arm_flag == 1:
                if user_input in ['m', 'u']:
                    mode = ord(user_input)
                    rospy.logwarn(f"{'MANUAL MODE' if user_input == 'm' else 'AUTONOMOUS MODE'}")
                
                elif user_input == 'p':
                    arm_flag = 0
                    mode = ord('m')
                    rospy.logwarn("VEHICLE DISARMED")
                
                else:
                    rospy.logerr("INVALID MODE")
            
            else:
                rospy.logerr("FIRST ARM THE VEHICLE")

            mode_data = [arm_flag, mode]
            msg = Int32MultiArray()
            msg.data = mode_data  # Send [arm_flag, mode]
            pub.publish(msg)

        else:
            rospy.logerr("Please enter only one character")

if __name__ == "__main__":
    change_commands()

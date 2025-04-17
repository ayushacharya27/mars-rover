#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import queue

ser = serial.Serial('/dev/rfcomm0', 9600)

prev_error = 0
prev_time = time.time()
integral = 0

window_size = 100
errors = []
times = []

# Real-time data queue
data_queue = queue.Queue()

plt.ion()
fig, ax = plt.subplots()
line_error, = ax.plot([], [], 'r-', label="Offset/Error")
line_target, = ax.plot([], [], 'b--', label="Target (0)")

ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.set_ylim(-300, 300)
ax.legend()
ax.grid(True)

start_time = time.time()

def pid_control(msg):
    global prev_error, prev_time, integral
    offset = msg.data

    kp = 0.4
    ki = 0
    kd = 0.1

    current_time = time.time()
    dt = current_time - prev_time
    derivative = (offset - prev_error) / dt
    integral += offset * dt

    corrector = kp * offset + kd * derivative + ki * integral
    send_str = f"{int(corrector)}\n"
    ser.write(send_str.encode())

    # Push the data into the queue for the main thread to pick up
    data_queue.put((current_time - start_time, offset))

    prev_error = offset
    prev_time = current_time

def update_plot():
    while not data_queue.empty():
        t, offset = data_queue.get()

        errors.append(offset)
        times.append(t)

        if len(times) > window_size:
            errors.pop(0)
            times.pop(0)

    if times:
        line_error.set_xdata(times)
        line_error.set_ydata(errors)
        line_target.set_xdata(times)
        line_target.set_ydata([0] * len(times))

        ax.set_xlim(times[0], times[-1])
        ax.figure.canvas.draw()
        ax.figure.canvas.flush_events()

if __name__ == "__main__":
    rospy.init_node("pid_commands_node", anonymous=True)
    rospy.Subscriber("aruco_center_distance", Float32, pid_control)

    rate = rospy.Rate(30)  # 30 Hz main loop rate

    while not rospy.is_shutdown():
        update_plot()
        plt.pause(0.001)
        rate.sleep()

#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import threading
import queue
from std_msgs.msg import Float32MultiArray

# Initialize position and trajectory lists
x, y, tx1, ty1 = 0.0, 0.0, 0.0, 0.0
trajectory = []

# Thread-safe queue for trajectory updates
data_queue = queue.Queue()

# Create a blank image for plotting
WIDTH, HEIGHT = 600, 600
CENTER = (WIDTH // 2, HEIGHT // 2)
SCALE = 20  # Scaling factor for visualization

def flow_points_callback(msg):
    """ROS callback to process optical flow data and update trajectory."""
    global x, y, tx1, ty1
    data = np.array(msg.data, dtype=np.float32)

    if len(data) != 40 or len(data) == 0:
        rospy.logerr("Must be 40 Points")
        return

    half = int(len(data) / 2)
    pts0 = data[:half].reshape(10, 2)
    pts1 = data[half:].reshape(10, 2)

    transformation_matrix, _ = cv2.estimateAffinePartial2D(pts0, pts1, method=cv2.RANSAC)

    if transformation_matrix is None:
        return

    tx, ty = transformation_matrix[:, 2]
    '''tx1 += tx
    ty1 += ty

    if tx1 <= -8 and ty1 <= -8:
        x += 1
        y += 1
        tx1 = 0
        ty1 = 0
    elif tx1 >= 8 or ty1 >= 8:
        x -= 1
        y -= 1

    # Add new position to queue
    data_queue.put((x, y))'''

    print(tx , ty)

def ros_thread():
    """Run ROS in a separate thread."""
    rospy.init_node("trajectory_node", anonymous=True)
    rospy.Subscriber("optical_flow_points", Float32MultiArray, flow_points_callback)
    rospy.spin()

def draw_trajectory():
    """Visualize the trajectory using OpenCV."""
    global trajectory
    canvas = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255  # White background

    # Draw the trajectory
    for i in range(1, len(trajectory)):
        pt1 = (int(CENTER[0] + trajectory[i-1][0] * SCALE), int(CENTER[1] - trajectory[i-1][1] * SCALE))
        pt2 = (int(CENTER[0] + trajectory[i][0] * SCALE), int(CENTER[1] - trajectory[i][1] * SCALE))
        cv2.line(canvas, pt1, pt2, (0, 0, 255), 2)

    # Draw the current position
    if trajectory:
        current_pos = (int(CENTER[0] + trajectory[-1][0] * SCALE), int(CENTER[1] - trajectory[-1][1] * SCALE))
        cv2.circle(canvas, current_pos, 5, (255, 0, 0), -1)

    cv2.imshow("Trajectory", canvas)
    cv2.waitKey(1)

if __name__ == "__main__":
    # Start ROS in a separate thread
    ros_thread = threading.Thread(target=ros_thread, daemon=True)
    ros_thread.start()

    while not rospy.is_shutdown():
        while not data_queue.empty():
            new_x, new_y = data_queue.get_nowait()
            trajectory.append((new_x, new_y))
        
        draw_trajectory()  # Update trajectory visualization

    cv2.destroyAllWindows()

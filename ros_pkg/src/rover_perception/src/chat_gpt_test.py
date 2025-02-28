#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import tf
from filterpy.kalman import KalmanFilter

# Initialize ROS Node
rospy.init_node("visual_odometry_kalman", anonymous=True)

# Publisher
pub_pose = rospy.Publisher("visual_odometry_pose", PoseStamped, queue_size=10)

# TF broadcaster
br = tf.TransformBroadcaster()

# Initialize global accumulated position
global total_x, total_y
total_x, total_y = 0.0, 0.0

# Initialize Kalman Filter for 2D position
kf = KalmanFilter(dim_x=4, dim_z=2)
kf.F = np.array([[1, 0, 1, 0],  # State transition matrix
                 [0, 1, 0, 1],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
kf.H = np.array([[1, 0, 0, 0],  # Measurement function
                 [0, 1, 0, 0]])
kf.P *= 1000.  # Initial covariance matrix
kf.R = np.array([[5, 0],        # Measurement noise
                 [0, 5]])
kf.Q = np.eye(4) * 0.1          # Process noise

# Initial state (x, y, vx, vy)
kf.x = np.array([0., 0., 0., 0.])

def flow_points_callback(msg):
    global kf, total_x, total_y

    data = np.array(msg.data, dtype=np.float32)

    if len(data) != 12:
        rospy.logwarn("Invalid data length. Expected 12 values but got {}".format(len(data)))
        return

    try:
        pts0 = data[:6].reshape(3, 2)
        pts1 = data[6:].reshape(3, 2)

        transformation_matrix = cv2.getAffineTransform(pts0, pts1)
        tx, ty = transformation_matrix[:, 2]

        # Accumulate displacement
        total_x += tx
        total_y += ty

        # Predict the next state
        kf.predict()

        # Update the filter with the new accumulated displacement
        z = np.array([total_x, total_y])
        kf.update(z)

        # Extract the filtered position
        x_filtered, y_filtered = kf.x[:2]

        # Publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = x_filtered
        pose_msg.pose.position.y = y_filtered
        pose_msg.pose.position.z = 0.0

        # Orientation (Assuming planar movement)
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        pub_pose.publish(pose_msg)

        # Broadcast TF
        br.sendTransform(
            (x_filtered, y_filtered, 0.0),
            quat,
            rospy.Time.now(),
            "base_link",
            "map"
        )

        rospy.loginfo(f"Published Pose: x={x_filtered:.2f}, y={y_filtered:.2f}")

    except Exception as e:
        rospy.logerr(f"Error processing optical flow points: {e}")

# Subscriber
rospy.Subscriber("optical_flow_points", Float32MultiArray, flow_points_callback)

rospy.spin()
#!/usr/bin/env python3
"""
Extended Kalman Filter Node
Implements EKF for robot state estimation combining motion model and measurements.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import numpy as np
from math import cos, sin, atan2, asin
import math
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class EKFNode(Node):
    """
    Extended Kalman Filter node for robot localization.
    Combines motion model predictions with IMU and visual odometry measurements.
    """
    
    def __init__(self):
        super().__init__('ekf_node')
        
        # Declare parameters
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('measurement_topic', '/measurement/odom')
        self.declare_parameter('ekf_odom_topic', '/ekf_odom')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('initial_covariance_xx', 0.1)
        self.declare_parameter('initial_covariance_yy', 0.1)
        self.declare_parameter('initial_covariance_tt', 0.1)
        
        # Get parameters
        motion_model_topic = self.get_parameter('motion_model_topic').get_parameter_value().string_value
        measurement_topic = self.get_parameter('measurement_topic').get_parameter_value().string_value
        ekf_odom_topic = self.get_parameter('ekf_odom_topic').get_parameter_value().string_value
        
        # State: [x, y, theta]
        self.state = np.array([
            self.get_parameter('initial_x').get_parameter_value().double_value,
            self.get_parameter('initial_y').get_parameter_value().double_value,
            self.get_parameter('initial_theta').get_parameter_value().double_value
        ])
        
        # Covariance matrix (3x3 for [x, y, theta])
        self.P = np.diag([
            self.get_parameter('initial_covariance_xx').get_parameter_value().double_value,
            self.get_parameter('initial_covariance_yy').get_parameter_value().double_value,
            self.get_parameter('initial_covariance_tt').get_parameter_value().double_value
        ])
        
        # Process noise covariance Q
        self.Q = np.eye(3) * 0.01
        
        # Measurement noise covariance R
        self.R = np.eye(3) * 0.1
        
        # Last prediction state
        self.last_prediction_state = None
        self.last_prediction_cov = None
        self.last_prediction_time = None
        
        # Publisher
        self.ekf_odom_pub = self.create_publisher(Odometry, ekf_odom_topic, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.motion_model_sub = self.create_subscription(
            Odometry,
            motion_model_topic,
            self.motion_model_callback,
            10
        )
        
        self.measurement_sub = self.create_subscription(
            Odometry,
            measurement_topic,
            self.measurement_callback,
            10
        )
        
        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_ekf_odom)  # 10 Hz
        
        self.get_logger().info('EKF Node started')
        self.get_logger().info(f'Initial state: x={self.state[0]:.3f}, y={self.state[1]:.3f}, theta={self.state[2]:.3f}')
    
    def motion_model_callback(self, msg):
        """
        Prediction step: Update state based on motion model.
        """
        # Extract predicted state from motion model
        pred_x = msg.pose.pose.position.x
        pred_y = msg.pose.pose.position.y
        pred_theta = self.quaternion_to_yaw(msg.pose.pose.orientation)
        
        # Extract covariance from motion model
        pred_cov = np.array(msg.pose.covariance).reshape(6, 6)
        pred_cov_3x3 = pred_cov[:3, :3]  # Extract [x, y, theta] part
        
        # Store prediction
        self.last_prediction_state = np.array([pred_x, pred_y, pred_theta])
        self.last_prediction_cov = pred_cov_3x3
        self.last_prediction_time = self.get_clock().now()
        
        # Update state with prediction (prediction step)
        self.state = self.last_prediction_state.copy()
        self.P = self.last_prediction_cov.copy()
    
    def measurement_callback(self, msg):
        """
        Update step: Correct state using measurement.
        """
        if self.last_prediction_state is None:
            self.get_logger().warn('No prediction available yet, skipping measurement update')
            return
        
        # Extract measurement
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        
        # Extract measurement covariance
        meas_cov = np.array(msg.pose.covariance).reshape(6, 6)
        R = meas_cov[:3, :3]  # Extract [x, y, theta] part
        
        # Measurement model: h(x) = [x, y, theta] (direct measurement)
        # For EKF, we need the Jacobian H = dh/dx
        # Since h(x) = x (identity), H = I
        H = np.eye(3)
        
        # Innovation (measurement residual)
        y = z - self.state
        
        # Normalize angle difference
        y[2] = math.atan2(sin(y[2]), cos(y[2]))
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        self.state[2] = math.atan2(sin(self.state[2]), cos(self.state[2]))  # Normalize angle
        
        # Update covariance (Joseph form for numerical stability)
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P @ (I - K @ H).T + K @ R @ K.T
        
        self.get_logger().debug(f'EKF update: state=[{self.state[0]:.3f}, {self.state[1]:.3f}, {self.state[2]:.3f}]')
    
    def publish_ekf_odom(self):
        """
        Publish EKF odometry and TF transform.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set pose
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.state[2])
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Set covariance (expand 3x3 to 6x6)
        cov_6x6 = np.zeros((6, 6))
        cov_6x6[:3, :3] = self.P
        odom_msg.pose.covariance = cov_6x6.flatten().tolist()
        
        # Publish odometry
        self.ekf_odom_pub.publish(odom_msg)
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
    
    @staticmethod
    def quaternion_to_yaw(quaternion):
        """
        Convert quaternion to yaw angle.
        """
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        """
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


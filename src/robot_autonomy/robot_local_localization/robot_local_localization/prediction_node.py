#!/usr/bin/env python3
"""
Prediction Node for Motion Model
Implements the prediction step of the EKF based on the skid-steering motion model.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
import numpy as np
from math import cos, sin
import math


class PredictionNode(Node):
    """
    Prediction node that implements the motion model for a skid-steering robot.
    Subscribes to cmd_vel and publishes predicted odometry based on motion model.
    """
    
    def __init__(self):
        super().__init__('prediction_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_separation', 0.45)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('prediction_topic', '/motion_model/odom')
        self.declare_parameter('process_noise_vx', 0.1)
        self.declare_parameter('process_noise_wz', 0.1)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        prediction_topic = self.get_parameter('prediction_topic').get_parameter_value().string_value
        self.process_noise_vx = self.get_parameter('process_noise_vx').get_parameter_value().double_value
        self.process_noise_wz = self.get_parameter('process_noise_wz').get_parameter_value().double_value
        
        # State: [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        self.last_time = None
        
        # Publisher
        self.prediction_pub = self.create_publisher(Odometry, prediction_topic, 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        # Timer for periodic prediction updates
        self.timer = self.create_timer(0.01, self.publish_prediction)  # 100 Hz
        
        self.get_logger().info('Prediction Node started')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}, Wheel separation: {self.wheel_separation}')
    
    def cmd_vel_callback(self, msg):
        """
        Update prediction based on cmd_vel command.
        Implements the motion model for skid-steering robot.
        """
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate time delta
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Extract velocities
        vx = msg.linear.x  # Linear velocity in m/s
        wz = msg.angular.z  # Angular velocity in rad/s
        
        # Motion model for skid-steering robot
        # x_{k+1} = x_k + vx * cos(theta_k) * dt
        # y_{k+1} = y_k + vx * sin(theta_k) * dt
        # theta_{k+1} = theta_k + wz * dt
        
        x, y, theta = self.state
        
        # Update state
        self.state[0] = x + vx * cos(theta) * dt
        self.state[1] = y + vx * sin(theta) * dt
        self.state[2] = theta + wz * dt
        
        # Normalize angle to [-pi, pi]
        self.state[2] = math.atan2(sin(self.state[2]), cos(self.state[2]))
        
        self.last_time = current_time
    
    def publish_prediction(self):
        """
        Publish the predicted odometry based on motion model.
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
        
        # Set covariance (process noise)
        # 3x3 covariance matrix for [x, y, theta]
        cov = np.zeros(36)
        cov[0] = self.process_noise_vx ** 2  # x variance
        cov[7] = self.process_noise_vx ** 2  # y variance
        cov[35] = self.process_noise_wz ** 2  # theta variance
        odom_msg.pose.covariance = cov.tolist()
        
        # Set twist (from last cmd_vel if available)
        # This would need to be stored from cmd_vel_callback
        odom_msg.twist.twist.linear.x = 0.0  # Will be updated by measurement
        odom_msg.twist.twist.angular.z = 0.0
        
        self.prediction_pub.publish(odom_msg)
    
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
    node = PredictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


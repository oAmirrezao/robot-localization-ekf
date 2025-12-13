#!/usr/bin/env python3
"""
Measurement Node
Combines IMU orientation and visual odometry measurements.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import numpy as np
from math import atan2, asin, cos, sin
import math


class MeasurementNode(Node):
    """
    Measurement node that combines IMU and visual odometry measurements.
    Subscribes to IMU and visual odometry topics, publishes combined measurement.
    """
    
    def __init__(self):
        super().__init__('measurement_node')
        
        # Declare parameters
        self.declare_parameter('imu_topic', '/zed/zed_node/imu/data_raw')
        self.declare_parameter('vo_topic', '/vo/odom')
        self.declare_parameter('measurement_topic', '/measurement/odom')
        self.declare_parameter('measurement_noise_dx', 0.05)
        self.declare_parameter('measurement_noise_dy', 0.05)
        self.declare_parameter('measurement_noise_theta', 0.1)
        
        # Get parameters
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        vo_topic = self.get_parameter('vo_topic').get_parameter_value().string_value
        measurement_topic = self.get_parameter('measurement_topic').get_parameter_value().string_value
        self.measurement_noise_dx = self.get_parameter('measurement_noise_dx').get_parameter_value().double_value
        self.measurement_noise_dy = self.get_parameter('measurement_noise_dy').get_parameter_value().double_value
        self.measurement_noise_theta = self.get_parameter('measurement_noise_theta').get_parameter_value().double_value
        
        # State variables
        self.last_vo_pose = None
        self.last_imu_orientation = None
        self.last_imu_time = None
        self.last_vo_time = None
        self.initial_imu_orientation = None
        self.initial_vo_pose = None
        
        # Publisher
        self.measurement_pub = self.create_publisher(Odometry, measurement_topic, 10)
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )
        
        self.vo_sub = self.create_subscription(
            Odometry,
            vo_topic,
            self.vo_callback,
            10
        )
        
        self.get_logger().info('Measurement Node started')
        self.get_logger().info(f'IMU topic: {imu_topic}, VO topic: {vo_topic}')
    
    def imu_callback(self, msg):
        """
        Store IMU orientation measurement.
        """
        self.last_imu_orientation = msg.orientation
        self.last_imu_time = self.get_clock().now()
        
        if self.initial_imu_orientation is None:
            self.initial_imu_orientation = msg.orientation
    
    def vo_callback(self, msg):
        """
        Process visual odometry and combine with IMU to create measurement.
        """
        if self.last_imu_orientation is None:
            self.get_logger().warn('IMU data not available yet')
            return
        
        # Get current VO pose
        current_vo_pose = msg.pose.pose
        
        # Initialize if first measurement
        if self.initial_vo_pose is None:
            self.initial_vo_pose = current_vo_pose
            self.last_vo_pose = current_vo_pose
            self.last_vo_time = self.get_clock().now()
            return
        
        # Calculate delta from visual odometry
        dx = current_vo_pose.position.x - self.initial_vo_pose.position.x
        dy = current_vo_pose.position.y - self.initial_vo_pose.position.y
        
        # Get orientation from IMU (relative to initial orientation)
        theta_imu = self.quaternion_to_yaw(self.last_imu_orientation)
        theta_initial = self.quaternion_to_yaw(self.initial_imu_orientation)
        dtheta = theta_imu - theta_initial
        
        # Normalize angle
        dtheta = math.atan2(sin(dtheta), cos(dtheta))
        
        # Create measurement odometry message
        measurement_msg = Odometry()
        measurement_msg.header.stamp = self.get_clock().now().to_msg()
        measurement_msg.header.frame_id = 'odom'
        measurement_msg.child_frame_id = 'base_link'
        
        # Set pose (absolute position from VO, orientation from IMU)
        measurement_msg.pose.pose.position.x = dx
        measurement_msg.pose.pose.position.y = dy
        measurement_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, dtheta)
        measurement_msg.pose.pose.orientation.x = qx
        measurement_msg.pose.pose.orientation.y = qy
        measurement_msg.pose.pose.orientation.z = qz
        measurement_msg.pose.pose.orientation.w = qw
        
        # Set covariance matrix (measurement noise)
        # 3x3 covariance for [dx, dy, dtheta]
        cov = np.zeros(36)
        cov[0] = self.measurement_noise_dx ** 2  # dx variance
        cov[7] = self.measurement_noise_dy ** 2  # dy variance
        cov[35] = self.measurement_noise_theta ** 2  # dtheta variance
        measurement_msg.pose.covariance = cov.tolist()
        
        # Set twist (can be zero or from VO if available)
        measurement_msg.twist.twist.linear.x = 0.0
        measurement_msg.twist.twist.linear.y = 0.0
        measurement_msg.twist.twist.angular.z = 0.0
        
        self.measurement_pub.publish(measurement_msg)
        
        # Update last pose
        self.last_vo_pose = current_vo_pose
        self.last_vo_time = self.get_clock().now()
    
    @staticmethod
    def quaternion_to_yaw(quaternion):
        """
        Convert quaternion to yaw angle.
        """
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = asin(sinp)
        
        # Yaw (z-axis rotation)
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
    node = MeasurementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


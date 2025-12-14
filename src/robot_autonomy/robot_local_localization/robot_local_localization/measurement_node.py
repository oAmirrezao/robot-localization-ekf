#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import numpy as np


class MeasurementNode(Node):
    def __init__(self):
        super().__init__('measurement_node')
        
        # Parameters
        self.declare_parameter('imu_topic', '/zed/zed_node/imu/data_raw')
        self.declare_parameter('vo_topic', '/vo/odom')
        self.declare_parameter('measurement_topic', '/measurement_model/pose')
        self.declare_parameter('frame_id', 'odom')
        
        imu_topic = self.get_parameter('imu_topic').value
        vo_topic = self.get_parameter('vo_topic').value
        measurement_topic = self.get_parameter('measurement_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Latest measurements
        self.latest_imu = None
        self.latest_vo = None
        self.last_time = self.get_clock().now()
        
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
        
        # Publisher
        self.measurement_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            measurement_topic,
            10
        )
        
        # Timer for combining measurements
        self.create_timer(0.05, self.combine_measurements)  # 20 Hz
        
        self.get_logger().info('Measurement Node started')

    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.latest_imu = msg

    def vo_callback(self, msg):
        """Store latest Visual Odometry data"""
        self.latest_vo = msg

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def combine_measurements(self):
        """Combine IMU and VO measurements"""
        current_time = self.get_clock().now()
        
        if self.latest_imu is None or self.latest_vo is None:
            return
        
        # Create combined measurement message
        measurement = PoseWithCovarianceStamped()
        measurement.header.stamp = current_time.to_msg()
        measurement.header.frame_id = self.frame_id
        
        # Position from Visual Odometry
        measurement.pose.pose.position.x = self.latest_vo.pose.pose.position.x
        measurement.pose.pose.position.y = self.latest_vo.pose.pose.position.y
        measurement.pose.pose.position.z = 0.0
        
        # Orientation from IMU (more accurate for yaw)
        measurement.pose.pose.orientation.x = self.latest_imu.orientation.x
        measurement.pose.pose.orientation.y = self.latest_imu.orientation.y
        measurement.pose.pose.orientation.z = self.latest_imu.orientation.z
        measurement.pose.pose.orientation.w = self.latest_imu.orientation.w
        
        # Combined covariance matrix
        # Position covariance from VO
        vo_cov_x = self.latest_vo.pose.covariance[0] if self.latest_vo.pose.covariance[0] > 0 else 0.01
        vo_cov_y = self.latest_vo.pose.covariance[7] if self.latest_vo.pose.covariance[7] > 0 else 0.01
        
        # Orientation covariance from IMU
        imu_cov_yaw = self.latest_imu.orientation_covariance[8] if self.latest_imu.orientation_covariance[8] > 0 else 0.001
        
        # Build covariance matrix (6x6 for pose)
        covariance = [0.0] * 36
        covariance[0] = vo_cov_x      # x variance
        covariance[7] = vo_cov_y      # y variance
        covariance[14] = 1e6          # z (not used, high uncertainty)
        covariance[21] = 1e6          # roll (not used)
        covariance[28] = 1e6          # pitch (not used)
        covariance[35] = imu_cov_yaw  # yaw variance
        
        measurement.pose.covariance = covariance
        
        # Publish combined measurement
        self.measurement_pub.publish(measurement)


def main(args=None):
    rclpy.init(args=args)
    node = MeasurementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

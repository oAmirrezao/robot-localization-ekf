#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math


class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')
        
        # Parameters
        self.declare_parameter('wheel_separation', 0.45)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        motion_model_topic = self.get_parameter('motion_model_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        
        # State: [x, y, theta]
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocities
        self.v = 0.0
        self.omega = 0.0
        
        # Time tracking
        self.last_time = self.get_clock().now()
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, motion_model_topic, 10)
        
        # Timer for prediction updates
        self.create_timer(0.05, self.prediction_update)  # 20 Hz
        
        self.get_logger().info('Prediction Node started')

    def cmd_vel_callback(self, msg):
        """Update commanded velocities"""
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def prediction_update(self):
        """Motion model prediction step"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0.0:
            return
        
        # Motion model: differential drive kinematics
        # x_dot = v * cos(theta)
        # y_dot = v * sin(theta)
        # theta_dot = omega
        
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.omega * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry
        self.publish_odometry(current_time)

    def publish_odometry(self, timestamp):
        """Publish motion model odometry"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Velocity
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.omega
        
        # Covariance (process noise)
        # Position covariance
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.05  # theta
        
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

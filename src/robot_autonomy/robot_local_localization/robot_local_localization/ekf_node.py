#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        # Parameters
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('measurement_topic', '/measurement_model/pose')
        self.declare_parameter('ekf_odom_topic', '/ekf/odom')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        
        motion_topic = self.get_parameter('motion_model_topic').value
        measurement_topic = self.get_parameter('measurement_topic').value
        ekf_topic = self.get_parameter('ekf_odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # EKF State: [x, y, theta]
        self.state = np.zeros(3)
        
        # Covariance matrix (3x3)
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance Q
        self.Q = np.diag([0.1, 0.1, 0.05])
        
        # Measurement noise covariance R
        self.R = np.diag([0.01, 0.01, 0.001])
        
        # Time tracking
        self.last_time = self.get_clock().now()
        self.initialized = False
        
        # Subscribers
        self.motion_sub = self.create_subscription(
            Odometry,
            motion_topic,
            self.motion_callback,
            10
        )
        
        self.measurement_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            measurement_topic,
            self.measurement_callback,
            10
        )
        
        # Publisher
        self.ekf_pub = self.create_publisher(Odometry, ekf_topic, 10)
        
        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('EKF Node started')

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def motion_callback(self, msg):
        """EKF Prediction step from motion model"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if not self.initialized:
            # Initialize state from first motion model
            self.state[0] = msg.pose.pose.position.x
            self.state[1] = msg.pose.pose.position.y
            self.state[2] = self.quaternion_to_yaw(msg.pose.pose.orientation)
            self.initialized = True
            self.get_logger().info(f'EKF initialized: x={self.state[0]:.2f}, y={self.state[1]:.2f}, theta={self.state[2]:.2f}')
            return
        
        if dt <= 0.0 or dt > 1.0:
            return
        
        # Get control inputs
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        # Prediction step
        self.predict(v, omega, dt)
        
        # Publish EKF odometry
        self.publish_odometry(current_time)

    def predict(self, v, omega, dt):
        """EKF Prediction Step"""
        theta = self.state[2]
        
        # State prediction
        self.state[0] += v * math.cos(theta) * dt
        self.state[1] += v * math.sin(theta) * dt
        self.state[2] += omega * dt
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Jacobian of motion model
        F = np.array([
            [1, 0, -v * math.sin(theta) * dt],
            [0, 1,  v * math.cos(theta) * dt],
            [0, 0,  1]
        ])
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q

    def measurement_callback(self, msg):
        """EKF Update step from measurements"""
        if not self.initialized:
            return
        
        # Extract measurement
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        
        # Measurement noise from covariance
        R = np.diag([
            max(msg.pose.covariance[0], 0.01),
            max(msg.pose.covariance[7], 0.01),
            max(msg.pose.covariance[35], 0.001)
        ])
        
        # Update step
        self.update(z, R)
        
        # Publish updated odometry
        current_time = self.get_clock().now()
        self.publish_odometry(current_time)

    def update(self, z, R):
        """EKF Update Step"""
        # Measurement model: H = I (direct measurement)
        H = np.eye(3)
        
        # Innovation
        y = z - self.state
        y[2] = self.normalize_angle(y[2])  # Normalize angle difference
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.state = self.state + K @ y
        self.state[2] = self.normalize_angle(self.state[2])
        
        # Covariance update
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

    def publish_odometry(self, timestamp):
        """Publish EKF odometry"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        # Position
        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        q = self.yaw_to_quaternion(self.state[2])
        odom.pose.pose.orientation.x = q['x']
        odom.pose.pose.orientation.y = q['y']
        odom.pose.pose.orientation.z = q['z']
        odom.pose.pose.orientation.w = q['w']
        
        # Covariance
        covariance = [0.0] * 36
        covariance[0] = self.P[0, 0]   # x
        covariance[7] = self.P[1, 1]   # y
        covariance[35] = self.P[2, 2]  # theta
        odom.pose.covariance = covariance
        
        # Publish
        self.ekf_pub.publish(odom)
        
        # Publish TF
        if self.publish_tf:
            self.publish_transform(timestamp, q)

    def publish_transform(self, timestamp, q):
        """Publish TF transform"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = q['x']
        t.transform.rotation.y = q['y']
        t.transform.rotation.z = q['z']
        t.transform.rotation.w = q['w']
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

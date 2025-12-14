#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import math


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        # Parameters
        self.declare_parameter('ekf_odom_topic', '/ekf/odom')
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('measurement_topic', '/measurement_model/pose')
        self.declare_parameter('wheel_odom_topic', '/wheel_encoder/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('angular_velocity', 0.5)
        self.declare_parameter('rectangle_width', 4.0)
        self.declare_parameter('rectangle_height', 2.0)
        
        ekf_topic = self.get_parameter('ekf_odom_topic').value
        motion_topic = self.get_parameter('motion_model_topic').value
        measurement_topic = self.get_parameter('measurement_topic').value
        wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        self.linear_vel = self.get_parameter('linear_velocity').value
        self.angular_vel = self.get_parameter('angular_velocity').value
        self.rect_width = self.get_parameter('rectangle_width').value
        self.rect_height = self.get_parameter('rectangle_height').value
        
        # Path storage
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'map'
        
        self.motion_path = Path()
        self.motion_path.header.frame_id = 'map'
        
        self.measurement_path = Path()
        self.measurement_path.header.frame_id = 'map'
        
        self.real_path = Path()
        self.real_path.header.frame_id = 'map'
        
        # Subscribers
        self.ekf_sub = self.create_subscription(
            Odometry,
            ekf_topic,
            self.ekf_callback,
            10
        )
        
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
        
        self.real_sub = self.create_subscription(
            Odometry,
            wheel_odom_topic,
            self.real_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        self.ekf_path_pub = self.create_publisher(Path, '/ekf/path', 10)
        self.motion_path_pub = self.create_publisher(Path, '/motion_model/path', 10)
        self.measurement_path_pub = self.create_publisher(Path, '/measurement_model/path', 10)
        self.real_path_pub = self.create_publisher(Path, '/real/path', 10)
        
        # State machine for rectangle trajectory
        self.state = 'IDLE'
        self.start_time = None
        self.segment_duration = 0.0
        self.current_segment = 0
        
        # Timer for control and publishing
        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5, self.publish_paths)
        
        # Start after delay
        self.create_timer(3.0, self.start_trajectory, clock=self.get_clock())
        
        self.get_logger().info('Test Node started. Will begin trajectory in 3 seconds...')

    def start_trajectory(self):
        """Start the rectangular trajectory"""
        if self.state == 'IDLE':
            self.state = 'MOVING_FORWARD'
            self.current_segment = 0
            self.start_time = self.get_clock().now()
            self.segment_duration = self.rect_width / self.linear_vel
            self.get_logger().info(f'Starting trajectory - Segment 0: Moving forward {self.rect_width}m')

    def ekf_callback(self, msg):
        """Store EKF path"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ekf_path.poses.append(pose)
        self.ekf_path.header.stamp = msg.header.stamp

    def motion_callback(self, msg):
        """Store motion model path"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.motion_path.poses.append(pose)
        self.motion_path.header.stamp = msg.header.stamp

    def measurement_callback(self, msg):
        """Store measurement model path"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.measurement_path.poses.append(pose)
        self.measurement_path.header.stamp = msg.header.stamp

    def real_callback(self, msg):
        """Store real (ground truth) path"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.real_path.poses.append(pose)
        self.real_path.header.stamp = msg.header.stamp

    def control_loop(self):
        """Control loop for rectangular trajectory"""
        if self.state == 'IDLE':
            return
        
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        cmd = Twist()
        
        if self.state == 'MOVING_FORWARD':
            if elapsed < self.segment_duration:
                cmd.linear.x = self.linear_vel
                cmd.angular.z = 0.0
            else:
                # Switch to turning
                self.state = 'TURNING'
                self.start_time = current_time
                self.segment_duration = (math.pi / 2.0) / self.angular_vel
                self.get_logger().info(f'Segment {self.current_segment} complete. Turning 90 degrees...')
        
        elif self.state == 'TURNING':
            if elapsed < self.segment_duration:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_vel
            else:
                # Next segment
                self.current_segment += 1
                
                if self.current_segment >= 8:  # 4 sides + 4 turns
                    self.state = 'COMPLETED'
                    self.get_logger().info('Rectangular trajectory completed!')
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                else:
                    self.state = 'MOVING_FORWARD'
                    self.start_time = current_time
                    
                    # Alternate between width and height
                    if self.current_segment % 2 == 0:
                        distance = self.rect_width
                    else:
                        distance = self.rect_height
                    
                    self.segment_duration = distance / self.linear_vel
                    self.get_logger().info(f'Segment {self.current_segment}: Moving forward {distance}m')
        
        elif self.state == 'COMPLETED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)

    def publish_paths(self):
        """Publish all paths for visualization"""
        current_time = self.get_clock().now()
        
        self.ekf_path.header.stamp = current_time.to_msg()
        self.ekf_path_pub.publish(self.ekf_path)
        
        self.motion_path.header.stamp = current_time.to_msg()
        self.motion_path_pub.publish(self.motion_path)
        
        self.measurement_path.header.stamp = current_time.to_msg()
        self.measurement_path_pub.publish(self.measurement_path)
        
        self.real_path.header.stamp = current_time.to_msg()
        self.real_path_pub.publish(self.real_path)


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

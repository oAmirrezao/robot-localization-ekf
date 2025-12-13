#!/usr/bin/env python3
"""
Test Node for Rectangular Path Following
Makes the robot follow a rectangular path and publishes paths for visualization.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
from enum import Enum


class PathState(Enum):
    """States for rectangular path following."""
    FORWARD_1 = 1
    TURN_1 = 2
    FORWARD_2 = 3
    TURN_2 = 4
    FORWARD_3 = 5
    TURN_3 = 6
    FORWARD_4 = 7
    TURN_4 = 8
    COMPLETE = 9


class TestNode(Node):
    """
    Test node that makes the robot follow a rectangular path.
    Publishes cmd_vel commands and visualizes paths in RViz.
    """
    
    def __init__(self):
        super().__init__('test_node')
        
        # Declare parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('ekf_odom_topic', '/ekf_odom')
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('measurement_topic', '/measurement/odom')
        self.declare_parameter('real_path_topic', '/real_path')
        self.declare_parameter('ekf_path_topic', '/ekf_path')
        self.declare_parameter('motion_model_path_topic', '/motion_model_path')
        self.declare_parameter('measurement_path_topic', '/measurement_path')
        self.declare_parameter('rectangle_length', 2.0)
        self.declare_parameter('rectangle_width', 1.5)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('use_sim_time', True)
        
        # Get parameters
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        ekf_odom_topic = self.get_parameter('ekf_odom_topic').get_parameter_value().string_value
        motion_model_topic = self.get_parameter('motion_model_topic').get_parameter_value().string_value
        measurement_topic = self.get_parameter('measurement_topic').get_parameter_value().string_value
        real_path_topic = self.get_parameter('real_path_topic').get_parameter_value().string_value
        ekf_path_topic = self.get_parameter('ekf_path_topic').get_parameter_value().string_value
        motion_model_path_topic = self.get_parameter('motion_model_path_topic').get_parameter_value().string_value
        measurement_path_topic = self.get_parameter('measurement_path_topic').get_parameter_value().string_value
        
        self.rectangle_length = self.get_parameter('rectangle_length').get_parameter_value().double_value
        self.rectangle_width = self.get_parameter('rectangle_width').get_parameter_value().double_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        
        # State variables
        self.state = PathState.FORWARD_1
        self.start_pose = None
        self.current_pose = None
        self.target_angle = 0.0
        self.distance_traveled = 0.0
        self.angle_turned = 0.0
        self.last_position = None
        
        # Path storage
        self.real_path = Path()
        self.real_path.header.frame_id = 'odom'
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom'
        self.motion_model_path = Path()
        self.motion_model_path.header.frame_id = 'odom'
        self.measurement_path = Path()
        self.measurement_path.header.frame_id = 'odom'
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.real_path_pub = self.create_publisher(Path, real_path_topic, 10)
        self.ekf_path_pub = self.create_publisher(Path, ekf_path_topic, 10)
        self.motion_model_path_pub = self.create_publisher(Path, motion_model_path_topic, 10)
        self.measurement_path_pub = self.create_publisher(Path, measurement_path_topic, 10)
        
        # Subscribers
        self.ekf_odom_sub = self.create_subscription(
            Odometry,
            ekf_odom_topic,
            self.ekf_odom_callback,
            10
        )
        
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
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        # Timer for path publishing
        self.path_timer = self.create_timer(0.2, self.publish_paths)  # 5 Hz
        
        self.get_logger().info('Test Node started')
        self.get_logger().info(f'Rectangle: {self.rectangle_length}m x {self.rectangle_width}m')
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def distance(self, p1, p2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    
    def ekf_odom_callback(self, msg):
        """Store EKF odometry."""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.ekf_path.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.ekf_path.poses) > 1000:
            self.ekf_path.poses.pop(0)
        
        # Update current pose for control
        self.current_pose = msg.pose.pose
    
    def motion_model_callback(self, msg):
        """Store motion model odometry."""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.motion_model_path.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.motion_model_path.poses) > 1000:
            self.motion_model_path.poses.pop(0)
    
    def measurement_callback(self, msg):
        """Store measurement odometry."""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.measurement_path.poses.append(pose_stamped)
        
        # Limit path length
        if len(self.measurement_path.poses) > 1000:
            self.measurement_path.poses.pop(0)
    
    def control_loop(self):
        """Main control loop for rectangular path following."""
        if self.current_pose is None:
            # If no odometry yet, wait a bit
            self.get_logger().debug('Waiting for odometry...')
            return
        
        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.get_logger().info('Starting rectangular path')
            self.last_position = self.current_pose.position
        
        # Calculate actual distance traveled
        if hasattr(self, 'last_position'):
            dx = self.current_pose.position.x - self.last_position.x
            dy = self.current_pose.position.y - self.last_position.y
            actual_distance = math.sqrt(dx*dx + dy*dy)
            self.distance_traveled += actual_distance
            self.last_position = self.current_pose.position
        else:
            self.last_position = self.current_pose.position
        
        cmd = Twist()
        
        if self.state == PathState.FORWARD_1:
            # Move forward for rectangle_length
            if self.distance_traveled < self.rectangle_length:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
            else:
                self.state = PathState.TURN_1
                self.distance_traveled = 0.0
                self.target_angle = self.quaternion_to_yaw(self.current_pose.orientation) + math.pi / 2
                self.angle_turned = 0.0
        
        elif self.state == PathState.TURN_1:
            # Turn 90 degrees
            current_angle = self.quaternion_to_yaw(self.current_pose.orientation)
            angle_diff = math.atan2(math.sin(self.target_angle - current_angle),
                                   math.cos(self.target_angle - current_angle))
            if abs(angle_diff) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            else:
                self.state = PathState.FORWARD_2
                self.distance_traveled = 0.0
        
        elif self.state == PathState.FORWARD_2:
            # Move forward for rectangle_width
            if self.distance_traveled < self.rectangle_width:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
            else:
                self.state = PathState.TURN_2
                self.distance_traveled = 0.0
                self.target_angle = self.quaternion_to_yaw(self.current_pose.orientation) + math.pi / 2
                self.angle_turned = 0.0
        
        elif self.state == PathState.TURN_2:
            # Turn 90 degrees
            current_angle = self.quaternion_to_yaw(self.current_pose.orientation)
            angle_diff = math.atan2(math.sin(self.target_angle - current_angle),
                                   math.cos(self.target_angle - current_angle))
            if abs(angle_diff) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            else:
                self.state = PathState.FORWARD_3
                self.distance_traveled = 0.0
        
        elif self.state == PathState.FORWARD_3:
            # Move forward for rectangle_length
            if self.distance_traveled < self.rectangle_length:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
            else:
                self.state = PathState.TURN_3
                self.distance_traveled = 0.0
                self.target_angle = self.quaternion_to_yaw(self.current_pose.orientation) + math.pi / 2
                self.angle_turned = 0.0
        
        elif self.state == PathState.TURN_3:
            # Turn 90 degrees
            current_angle = self.quaternion_to_yaw(self.current_pose.orientation)
            angle_diff = math.atan2(math.sin(self.target_angle - current_angle),
                                   math.cos(self.target_angle - current_angle))
            if abs(angle_diff) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            else:
                self.state = PathState.FORWARD_4
                self.distance_traveled = 0.0
        
        elif self.state == PathState.FORWARD_4:
            # Move forward for rectangle_width
            if self.distance_traveled < self.rectangle_width:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
            else:
                self.state = PathState.TURN_4
                self.distance_traveled = 0.0
                self.target_angle = self.quaternion_to_yaw(self.current_pose.orientation) + math.pi / 2
                self.angle_turned = 0.0
        
        elif self.state == PathState.TURN_4:
            # Turn 90 degrees to complete rectangle
            current_angle = self.quaternion_to_yaw(self.current_pose.orientation)
            angle_diff = math.atan2(math.sin(self.target_angle - current_angle),
                                   math.cos(self.target_angle - current_angle))
            if abs(angle_diff) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
            else:
                self.state = PathState.COMPLETE
                self.get_logger().info('Rectangular path completed!')
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        
        elif self.state == PathState.COMPLETE:
            # Stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        # Add current pose to real path (using EKF as ground truth proxy)
        if self.current_pose:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.pose = self.current_pose
            self.real_path.poses.append(pose_stamped)
            
            # Limit path length
            if len(self.real_path.poses) > 1000:
                self.real_path.poses.pop(0)
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_paths(self):
        """Publish all paths for visualization."""
        now = self.get_clock().now().to_msg()
        
        # Update headers
        self.real_path.header.stamp = now
        self.ekf_path.header.stamp = now
        self.motion_model_path.header.stamp = now
        self.measurement_path.header.stamp = now
        
        # Publish paths
        self.real_path_pub.publish(self.real_path)
        self.ekf_path_pub.publish(self.ekf_path)
        self.motion_model_path_pub.publish(self.motion_model_path)
        self.measurement_path_pub.publish(self.measurement_path)


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


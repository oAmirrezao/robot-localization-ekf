#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import math
import tf_transformations # مطمئن شوید این پکیج نصب است (sudo apt install ros-humble-tf-transformations)
# اگر tf_transformations نصب نیست، از فرمول ریاضی ساده پایین استفاده می‌کنیم.

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # Parameters
        self.declare_parameter('ekf_odom_topic', '/ekf/odom')
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('measurement_topic', '/measurement_model/pose')
        self.declare_parameter('wheel_odom_topic', '/wheel_encoder/odom') # این تاپیک باید در گازبو فعال باشد
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_velocity', 0.4)
        self.declare_parameter('angular_velocity', 0.3)
        self.declare_parameter('rectangle_width', 4.0)
        self.declare_parameter('rectangle_height', 2.0)

        ekf_topic = self.get_parameter('ekf_odom_topic').value
        motion_topic = self.get_parameter('motion_model_topic').value
        measurement_topic = self.get_parameter('measurement_topic').value
        wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.target_linear_vel = self.get_parameter('linear_velocity').value
        self.target_angular_vel = self.get_parameter('angular_velocity').value
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

        # Current Robot State from EKF
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.ekf_received = False

        # Subscribers
        self.ekf_sub = self.create_subscription(Odometry, ekf_topic, self.ekf_callback, 10)
        self.motion_sub = self.create_subscription(Odometry, motion_topic, self.motion_callback, 10)
        self.measurement_sub = self.create_subscription(PoseWithCovarianceStamped, measurement_topic, self.measurement_callback, 10)
        # برای رسم مسیر واقعی (Ground Truth) اگر گازبو منتشر کند
        self.real_sub = self.create_subscription(Odometry, wheel_odom_topic, self.real_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.ekf_path_pub = self.create_publisher(Path, '/ekf/path', 10)
        self.motion_path_pub = self.create_publisher(Path, '/motion_model/path', 10)
        self.measurement_path_pub = self.create_publisher(Path, '/measurement_model/path', 10)
        self.real_path_pub = self.create_publisher(Path, '/real/path', 10)

        # State Machine
        self.state = 'IDLE'
        self.current_segment = 0
        
        # Variables for Closed-Loop Control
        self.start_pose_x = 0.0
        self.start_pose_y = 0.0
        self.start_pose_yaw = 0.0
        self.target_distance = 0.0
        self.target_angle = 0.0

        self.create_timer(0.1, self.control_loop)
        self.create_timer(0.5, self.publish_paths)
        
        # Wait a bit for EKF to stabilize before starting
        self.create_timer(5.0, self.start_trajectory)

        self.get_logger().info('Test Node initialized. Waiting for EKF...')

    def start_trajectory(self):
        if self.state == 'IDLE' and self.ekf_received:
            self.get_logger().info('Starting Trajectory...')
            self.setup_segment()

    def setup_segment(self):
        """تنظیم متغیرهای کنترلی برای شروع سگمنت جدید"""
        self.start_pose_x = self.current_x
        self.start_pose_y = self.current_y
        self.start_pose_yaw = self.current_yaw
        
        if self.current_segment >= 8: # 4 sides + 4 turns
            self.state = 'COMPLETED'
            return

        if self.current_segment % 2 == 0: # حرکت مستقیم (0, 2, 4, 6)
            self.state = 'MOVING_FORWARD'
            # تعیین طول ضلع (یک در میان طول و عرض)
            side_index = self.current_segment // 2
            if side_index % 2 == 0:
                self.target_distance = self.rect_width
            else:
                self.target_distance = self.rect_height
            self.get_logger().info(f'Segment {self.current_segment}: Forward {self.target_distance}m')
            
        else: # چرخش (1, 3, 5, 7)
            self.state = 'TURNING'
            self.target_angle = math.pi / 2.0 # 90 degrees
            self.get_logger().info(f'Segment {self.current_segment}: Turn 90 deg')

    def get_yaw_from_pose(self, pose):
        # تبدیل کواتریونیون به زاویه Yaw
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def ekf_callback(self, msg):
        self.ekf_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_pose(msg.pose.pose)

        # Path Visualization
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ekf_path.poses.append(pose)
        self.ekf_path.header.stamp = self.get_clock().now().to_msg() # Update stamp for TF

    def motion_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.motion_path.poses.append(pose)
        self.motion_path.header.stamp = self.get_clock().now().to_msg()

    def measurement_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.measurement_path.poses.append(pose)
        self.measurement_path.header.stamp = self.get_clock().now().to_msg()
    
    def real_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.real_path.poses.append(pose)
        self.real_path.header.stamp = self.get_clock().now().to_msg()

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle <= -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.ekf_received or self.state == 'IDLE':
            return

        cmd = Twist()

        if self.state == 'MOVING_FORWARD':
            # محاسبه فاصله طی شده نسبت به نقطه شروع سگمنت
            dx = self.current_x - self.start_pose_x
            dy = self.current_y - self.start_pose_y
            dist_traveled = math.sqrt(dx*dx + dy*dy)

            error = self.target_distance - dist_traveled

            if error > 0.05: # تلورانس 5 سانتیمتر
                # کنترلر تناسبی ساده (P-Controller) برای نرم ایستادن
                cmd.linear.x = min(self.target_linear_vel, error * 2.0)
                cmd.linear.x = max(cmd.linear.x, 0.1) # حداقل سرعت
                
                # تصحیح زاویه برای حرکت مستقیم
                angle_diff = self.normalize_angle(self.start_pose_yaw - self.current_yaw)
                cmd.angular.z = angle_diff * 1.0 # P-gain for angle correction
            else:
                # رسیدن به هدف
                cmd.linear.x = 0.0
                self.current_segment += 1
                self.setup_segment()

        elif self.state == 'TURNING':
            # محاسبه زاویه چرخش
            angle_diff = self.normalize_angle(self.current_yaw - self.start_pose_yaw)
            # چون 90 درجه مثبت (چپ) می‌خواهیم
            remaining = self.target_angle - abs(angle_diff)

            if remaining > 0.02: # تلورانس حدود 1 درجه
                speed = min(self.target_angular_vel, remaining * 2.0)
                speed = max(speed, 0.1)
                cmd.angular.z = speed
            else:
                cmd.angular.z = 0.0
                self.current_segment += 1
                self.setup_segment()

        elif self.state == 'COMPLETED':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Rectangle Completed!', once=True)

        self.cmd_vel_pub.publish(cmd)

    def publish_paths(self):
        self.ekf_path_pub.publish(self.ekf_path)
        self.motion_path_pub.publish(self.motion_path)
        self.measurement_path_pub.publish(self.measurement_path)
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

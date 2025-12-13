# پروژه تخمین وضعیت ربات موبایل با استفاده از فیلتر کالمن توسعه یافته (EKF)

## 📋 معرفی پروژه

این پروژه یک سیستم تخمین موقعیت مکانی (Localization) برای ربات موبایل چهار چرخ است که از **Extended Kalman Filter (EKF)** برای ترکیب داده‌های حسگرهای مختلف استفاده می‌کند:

- **Wheel Encoders**: داده‌های اودومتری چرخ‌ها
- **Visual Odometry (VO)**: تخمین حرکت از دوربین RGBD
- **IMU**: جهت‌گیری از سنسور اینرسی

### ویژگی‌های پروژه

✅ پیاده‌سازی کامل EKF با مدل حرکت و اندازه‌گیری  
✅ کنترل مستقل موتورها از طریق دستورات RPM  
✅ ترکیب داده‌های Visual Odometry و IMU  
✅ مسیر مستطیلی خودکار برای تست  
✅ مقایسه بصری 4 روش تخمین موقعیت در RViz  
✅ قابلیت ضبط داده با Rosbag  

---

## 🛠️ پیش‌نیازها

### نرم‌افزارهای مورد نیاز

- **سیستم عامل**: Ubuntu 22.04 (یا WSL2 با Ubuntu 22.04)
- **ROS2**: Humble Hawksbill
- **Gazebo**: Ignition Gazebo (Fortress یا بالاتر)
- **Python**: 3.10+
- **کامپایلر**: GCC 11+

### بسته‌های ROS2

```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-rtabmap-ros \
    ros-humble-navigation2
```

### کتابخانه‌های Python

```bash
sudo apt-get install -y \
    python3-numpy \
    python3-transforms3d \
    python3-tf-transformations
```

---

## 📁 ساختار پروژه

```
robotic_course-main/
├── src/
│   ├── robot_description/              # بسته اصلی ربات (موجود)
│   │   ├── scripts/
│   │   │   ├── cmd_vel_controller.cpp  # [جدید] تبدیل cmd_vel به RPM
│   │   │   ├── ekf_diff_imu.cpp
│   │   │   ├── frame_id_converter.cpp
│   │   │   └── motor_command.cpp
│   │   ├── src/
│   │   │   └── description/
│   │   │       └── robot.urdf          # [اصلاح شده] کنترل موتور مستقل
│   │   ├── CMakeLists.txt              # [اصلاح شده]
│   │   └── ...
│   │
│   └── robot_autonomy/                 # [جدید] پکیج‌های خودمختاری
│       └── robot_local_localization/   # [جدید] بسته تخمین موقعیت
│           ├── robot_local_localization/
│           │   ├── __init__.py
│           │   ├── prediction_node.py      # مرحله پیش‌بینی EKF
│           │   ├── measurement_node.py     # ترکیب VO و IMU
│           │   ├── ekf_node.py             # اجرای کامل EKF
│           │   └── test_node.py            # تست مسیر مستطیلی
│           ├── resource/
│           │   └── robot_local_localization
│           ├── package.xml
│           └── setup.py
```

---

## 🚀 نصب و راه‌اندازی

### مرحله 1: کلون کردن یا آماده‌سازی Workspace

اگر workspace موجود دارید:

```bash
cd ~/robotic_course-main
```

اگر از ابتدا شروع می‌کنید:

```bash
mkdir -p ~/robotic_course-main/src
cd ~/robotic_course-main/src
# بسته robot_description موجود را اینجا قرار دهید
```

### مرحله 2: ایجاد ساختار بسته جدید

```bash
cd ~/robotic_course-main/src
mkdir -p robot_autonomy/robot_local_localization/robot_local_localization
mkdir -p robot_autonomy/robot_local_localization/resource
mkdir -p robot_autonomy/robot_local_localization/test

# ایجاد فایل‌های خالی
touch robot_autonomy/robot_local_localization/robot_local_localization/__init__.py
touch robot_autonomy/robot_local_localization/resource/robot_local_localization
```

### مرحله 3: اضافه کردن فایل‌های پایتون

فایل‌های زیر را در مسیر `robot_autonomy/robot_local_localization/robot_local_localization/` ایجاد کنید:

#### 📄 `prediction_node.py`

<details>
<summary>کلیک کنید تا کد را ببینید</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
import math

class PredictionNode(Node):
    def __init__(self):
        super().__init__('prediction_node')
        
        # پارامترها
        self.declare_parameter('wheel_separation', 0.45)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('sigma_v', 0.1)
        self.declare_parameter('sigma_omega', 0.1)
        self.declare_parameter('update_rate', 50.0)
        
        # دریافت پارامترها
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.motion_model_topic = self.get_parameter('motion_model_topic').value
        self.sigma_v = self.get_parameter('sigma_v').value
        self.sigma_omega = self.get_parameter('sigma_omega').value
        update_rate = self.get_parameter('update_rate').value
        
        # حالت: [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])
        self.covariance = np.eye(3) * 0.1
        
        # ورودی کنترل
        self.v = 0.0
        self.omega = 0.0
        
        self.last_time = self.get_clock().now()
        
        # Subscriber و Publisher
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        
        self.motion_model_pub = self.create_publisher(
            Odometry, self.motion_model_topic, 10)
        
        self.timer = self.create_timer(1.0 / update_rate, self.prediction_update)
        
        self.get_logger().info('Prediction Node آماده است')
    
    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z
    
    def wrap_to_pi(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def prediction_update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt <= 0.0 or dt > 1.0:
            dt = 0.02
        
        x, y, theta = self.state
        v = self.v
        omega = self.omega
        
        # پیش‌بینی حالت
        x_pred = x + v * math.cos(theta) * dt
        y_pred = y + v * math.sin(theta) * dt
        theta_pred = self.wrap_to_pi(theta + omega * dt)
        
        self.state = np.array([x_pred, y_pred, theta_pred])
        
        # ژاکوبین F
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0]
        ])
        
        # ژاکوبین G
        G = np.array([
            [math.cos(theta) * dt, 0.0],
            [math.sin(theta) * dt, 0.0],
            [0.0, dt]
        ])
        
        Q_u = np.array([
            [self.sigma_v**2, 0.0],
            [0.0, self.sigma_omega**2]
        ])
        
        Q = G @ Q_u @ G.T
        self.covariance = F @ self.covariance @ F.T + Q
        
        self.publish_odometry()
    
    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0
        
        theta = self.state[2]
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = self.covariance[0, 0]
        odom_msg.pose.covariance[7] = self.covariance[1, 1]
        odom_msg.pose.covariance[35] = self.covariance[2, 2]
        
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega
        
        self.motion_model_pub.publish(odom_msg)

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
```
</details>

#### 📄 `measurement_node.py`

<details>
<summary>کلیک کنید تا کد را ببینید</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
from tf_transformations import euler_from_quaternion

class MeasurementNode(Node):
    def __init__(self):
        super().__init__('measurement_node')
        
        self.declare_parameter('imu_topic', '/zed/zed_node/imu/data_raw')
        self.declare_parameter('vo_topic', '/vo/odom')
        self.declare_parameter('measurement_topic', '/measurement_model')
        self.declare_parameter('sigma_dx', 0.05)
        self.declare_parameter('sigma_dy', 0.05)
        self.declare_parameter('sigma_theta', 0.01)
        
        self.imu_topic = self.get_parameter('imu_topic').value
        self.vo_topic = self.get_parameter('vo_topic').value
        self.measurement_topic = self.get_parameter('measurement_topic').value
        self.sigma_dx = self.get_parameter('sigma_dx').value
        self.sigma_dy = self.get_parameter('sigma_dy').value
        self.sigma_theta = self.get_parameter('sigma_theta').value
        
        self.latest_imu = None
        self.latest_vo = None
        self.prev_vo_x = 0.0
        self.prev_vo_y = 0.0
        self.vo_initialized = False
        
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.vo_sub = self.create_subscription(Odometry, self.vo_topic, self.vo_callback, 10)
        
        self.measurement_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.measurement_topic, 10)
        
        self.get_logger().info('Measurement Node آماده است')
    
    def imu_callback(self, msg):
        self.latest_imu = msg
        self.publish_combined_measurement()
    
    def vo_callback(self, msg):
        self.latest_vo = msg
        
        if not self.vo_initialized:
            self.prev_vo_x = msg.pose.pose.position.x
            self.prev_vo_y = msg.pose.pose.position.y
            self.vo_initialized = True
        
        self.publish_combined_measurement()
    
    def publish_combined_measurement(self):
        if self.latest_imu is None or self.latest_vo is None:
            return
        
        orientation_q = self.latest_imu.orientation
        orientation_list = [orientation_q.x, orientation_q.y, 
                          orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta = yaw
        
        current_vo_x = self.latest_vo.pose.pose.position.x
        current_vo_y = self.latest_vo.pose.pose.position.y
        
        dx = current_vo_x - self.prev_vo_x
        dy = current_vo_y - self.prev_vo_y
        
        self.prev_vo_x = current_vo_x
        self.prev_vo_y = current_vo_y
        
        measurement = PoseWithCovarianceStamped()
        measurement.header.stamp = self.get_clock().now().to_msg()
        measurement.header.frame_id = 'odom'
        
        measurement.pose.pose.position.x = dx
        measurement.pose.pose.position.y = dy
        measurement.pose.pose.position.z = 0.0
        
        measurement.pose.pose.orientation.x = 0.0
        measurement.pose.pose.orientation.y = 0.0
        measurement.pose.pose.orientation.z = math.sin(theta / 2.0)
        measurement.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        measurement.pose.covariance = [0.0] * 36
        measurement.pose.covariance[0] = self.sigma_dx**2
        measurement.pose.covariance[7] = self.sigma_dy**2
        measurement.pose.covariance[35] = self.sigma_theta**2
        
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
```
</details>

#### 📄 `ekf_node.py`

<details>
<summary>کلیک کنید تا کد را ببینید</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from tf_transformations import euler_from_quaternion

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        self.declare_parameter('motion_model_topic', '/motion_model/odom')
        self.declare_parameter('measurement_topic', '/measurement_model')
        self.declare_parameter('ekf_odom_topic', '/ekf/odom')
        self.declare_parameter('publish_tf', True)
        
        self.motion_model_topic = self.get_parameter('motion_model_topic').value
        self.measurement_topic = self.get_parameter('measurement_topic').value
        self.ekf_odom_topic = self.get_parameter('ekf_odom_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.state = np.array([0.0, 0.0, 0.0])
        self.covariance = np.eye(3) * 0.1
        
        self.motion_model_sub = self.create_subscription(
            Odometry, self.motion_model_topic, self.motion_model_callback, 10)
        
        self.measurement_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.measurement_topic, 
            self.measurement_callback, 10)
        
        self.ekf_odom_pub = self.create_publisher(Odometry, self.ekf_odom_topic, 10)
        
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('EKF Node آماده است')
    
    def wrap_to_pi(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle <= -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def motion_model_callback(self, msg):
        x_pred = msg.pose.pose.position.x
        y_pred = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, 
                          orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta_pred = yaw
        
        P_pred = np.zeros((3, 3))
        P_pred[0, 0] = msg.pose.covariance[0]
        P_pred[1, 1] = msg.pose.covariance[7]
        P_pred[2, 2] = msg.pose.covariance[35]
        
        self.state = np.array([x_pred, y_pred, theta_pred])
        self.covariance = P_pred
        
        self.publish_ekf_state()
    
    def measurement_callback(self, msg):
        dx_meas = msg.pose.pose.position.x
        dy_meas = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, 
                          orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        theta_meas = yaw
        
        R = np.zeros((3, 3))
        R[0, 0] = msg.pose.covariance[0]
        R[1, 1] = msg.pose.covariance[7]
        R[2, 2] = msg.pose.covariance[35]
        
        z_meas = np.array([dx_meas, dy_meas, theta_meas])
        z_pred = np.array([0.0, 0.0, self.state[2]])
        
        innovation = z_meas - z_pred
        innovation[2] = self.wrap_to_pi(innovation[2])
        
        H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        S = H @ self.covariance @ H.T + R
        
        try:
            K = self.covariance @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn('ماتریس کوواریانس نوآوری تکین است')
            return
        
        self.state[0] += K[0, 0] * innovation[0] + K[0, 1] * innovation[1]
        self.state[1] += K[1, 0] * innovation[0] + K[1, 1] * innovation[1]
        self.state[2] += K[2, 2] * innovation[2]
        self.state[2] = self.wrap_to_pi(self.state[2])
        
        I = np.eye(3)
        self.covariance = (I - K @ H) @ self.covariance
        
        self.publish_ekf_state()
    
    def publish_ekf_state(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0
        
        theta = self.state[2]
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = self.covariance[0, 0]
        odom_msg.pose.covariance[7] = self.covariance[1, 1]
        odom_msg.pose.covariance[35] = self.covariance[2, 2]
        
        self.ekf_odom_pub.publish(odom_msg)
        
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link_ekf'
            
            t.transform.translation.x = self.state[0]
            t.transform.translation.y = self.state[1]
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(theta / 2.0)
            t.transform.rotation.w = math.cos(theta / 2.0)
            
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
```
</details>

#### 📄 `test_node.py`

<details>
<summary>کلیک کنید تا کد را ببینید</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        self.declare_parameter('rectangle_length', 2.0)
        self.declare_parameter('rectangle_width', 1.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.3)
        
        self.rect_length = self.get_parameter('rectangle_length').value
        self.rect_width = self.get_parameter('rectangle_width').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        
        self.state = 'IDLE'
        self.segment = 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'odom'
        self.ground_truth_path = Path()
        self.ground_truth_path.header.frame_id = 'odom'
        self.motion_model_path = Path()
        self.motion_model_path.header.frame_id = 'odom'
        self.measurement_path = Path()
        self.measurement_path.header.frame_id = 'odom'
        
        self.meas_x = 0.0
        self.meas_y = 0.0
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ekf_path_pub = self.create_publisher(Path, '/paths/ekf', 10)
        self.gt_path_pub = self.create_publisher(Path, '/paths/ground_truth', 10)
        self.mm_path_pub = self.create_publisher(Path, '/paths/motion_model', 10)
        self.meas_path_pub = self.create_publisher(Path, '/paths/measurement', 10)
        
        self.ekf_sub = self.create_subscription(Odometry, '/ekf/odom', self.ekf_callback, 10)
        self.gt_sub = self.create_subscription(Odometry, '/wheel_encoder/odom', self.ground_truth_callback, 10)
        self.mm_sub = self.create_subscription(Odometry, '/motion_model/odom', self.motion_model_callback, 10)
        self.meas_sub = self.create_subscription(PoseWithCovarianceStamped, '/measurement_model', self.measurement_callback, 10)
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_paths)
        self.start_timer = self.create_timer(2.0, self.start_rectangle)
        
        self.get_logger().info('Test Node آماده است - 2 ثانیه تا شروع...')
    
    def start_rectangle(self):
        self.start_timer.cancel()
        self.state = 'FORWARD'
        self.segment = 0
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_theta = self.current_theta
        self.get_logger().info('شروع مسیر مستطیلی')
    
    def ekf_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ekf_path.poses.append(pose)
    
    def ground_truth_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ground_truth_path.poses.append(pose)
    
    def motion_model_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.motion_model_path.poses.append(pose)
    
    def measurement_callback(self, msg):
        dx = msg.pose.pose.position.x
        dy = msg.pose.pose.position.y
        
        self.meas_x += dx
        self.meas_y += dy
        
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = self.meas_x
        pose.pose.position.y = self.meas_y
        pose.pose.position.z = 0.0
        pose.pose.orientation = msg.pose.pose.orientation
        self.measurement_path.poses.append(pose)
    
    def control_loop(self):
        if self.state == 'IDLE':
            return
        
        cmd = Twist()
        
        if self.state == 'FORWARD':
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
            
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if self.segment % 2 == 0:
                target_distance = self.rect_length
            else:
                target_distance = self.rect_width
            
            if distance >= target_distance:
                self.state = 'TURN'
                self.start_theta = self.current_theta
                self.get_logger().info(f'بخش {self.segment} تمام شد، شروع چرخش')
        
        elif self.state == 'TURN':
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            
            angle_diff = self.current_theta - self.start_theta
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            if abs(angle_diff) >= math.pi / 2:
                self.segment += 1
                if self.segment >= 4:
                    self.state = 'IDLE'
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.get_logger().info('مسیر مستطیلی تمام شد!')
                else:
                    self.state = 'FORWARD'
                    self.start_x = self.current_x
                    self.start_y = self.current_y
                    self.get_logger().info(f'چرخش تمام شد، شروع بخش {self.segment}')
        
        self.cmd_vel_pub.publish(cmd)
    
    def publish_paths(self):
        timestamp = self.get_clock().now().to_msg()
        
        self.ekf_path.header.stamp = timestamp
        self.ground_truth_path.header.stamp = timestamp
        self.motion_model_path.header.stamp = timestamp
        self.measurement_path.header.stamp = timestamp
        
        self.ekf_path_pub.publish(self.ekf_path)
        self.gt_path_pub.publish(self.ground_truth_path)
        self.mm_path_pub.publish(self.motion_model_path)
        self.meas_path_pub.publish(self.measurement_path)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
</details>

پس از ایجاد، executable کنید:

```bash
cd ~/robotic_course-main/src/robot_autonomy/robot_local_localization/robot_local_localization
chmod +x *.py
```

### مرحله 4: ایجاد فایل‌های پیکربندی بسته

#### 📄 `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_local_localization</name>
  <version>0.0.1</version>
  <description>EKF-based localization for mobile robot</description>
  <maintainer email="student@example.com">Student</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-transforms3d</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 📄 `setup.py`

```python
from setuptools import setup

package_name = 'robot_local_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='EKF-based localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prediction_node = robot_local_localization.prediction_node:main',
            'measurement_node = robot_local_localization.measurement_node:main',
            'ekf_node = robot_local_localization.ekf_node:main',
            'test_node = robot_local_localization.test_node:main',
        ],
    },
)
```

### مرحله 5: اصلاح بسته robot_description

#### 📄 ایجاد `cmd_vel_controller.cpp`

در مسیر `robot_description/scripts/` فایل جدید `cmd_vel_controller.cpp` ایجاد کنید:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>

class CmdVelController : public rclcpp::Node {
public:
    CmdVelController() : Node("cmd_vel_controller") {
        this->declare_parameter<double>("wheel_radius", 0.1);
        this->declare_parameter<double>("wheel_separation", 0.45);
        
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        
        left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/left_motor_rpm", 10);
        right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/right_motor_rpm", 10);
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelController::cmdVelCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "CMD_VEL Controller شروع شد");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v = msg->linear.x;
        double omega = msg->angular.z;
        
        double v_left = v - (omega * wheel_separation_ / 2.0);
        double v_right = v + (omega * wheel_separation_ / 2.0);
        
        double omega_left = v_left / wheel_radius_;
        double omega_right = v_right / wheel_radius_;
        
        double rpm_left = omega_left * 60.0 / (2.0 * M_PI);
        double rpm_right = omega_right * 60.0 / (2.0 * M_PI);
        
        std_msgs::msg::Float64 left_msg, right_msg;
        left_msg.data = rpm_left;
        right_msg.data = rpm_right;
        
        left_motor_pub_->publish(left_msg);
        right_motor_pub_->publish(right_msg);
    }
    
    double wheel_radius_, wheel_separation_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelController>());
    rclcpp::shutdown();
    return 0;
}
```

#### 📄 اصلاح `CMakeLists.txt`

به فایل `robot_description/CMakeLists.txt` اضافه کنید:

```cmake
# پیدا کردن dependency های cmd_vel_controller (احتمالاً قبلاً موجود است)
find_package(geometry_msgs REQUIRED)

# اضافه کردن executable
add_executable(cmd_vel_controller_node scripts/cmd_vel_controller.cpp)
ament_target_dependencies(cmd_vel_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

# نصب executable
install(TARGETS
  cmd_vel_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
```

#### 📄 اصلاح `robot.urdf`

در فایل `robot_description/src/description/robot.urdf`، بخش Gazebo plugins را پیدا کنید و تغییرات زیر را اعمال کنید:

**حذف کنید** (plugin DiffDrive):
```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system"
      name="gz::sim::systems::DiffDrive">
    <!-- ... -->
  </plugin>
</gazebo>
```

**اضافه کنید** (کنترل موتورهای مستقل):
```xml
<!-- کنترل موتور چپ -->
<gazebo>
  <plugin filename="gz-sim-joint-controller-system"
      name="gz::sim::systems::JointController">
      <joint_name>drivewhl_l_joint</joint_name>
      <initial_velocity>0.0</initial_velocity>
      <topic>left_motor_rpm</topic>
      <p_gain>0.5</p_gain>
      <i_gain>0.1</i_gain>
  </plugin>
</gazebo>

<!-- کنترل موتور راست -->
<gazebo>
  <plugin filename="gz-sim-joint-controller-system"
      name="gz::sim::systems::JointController">
      <joint_name>drivewhl_r_joint</joint_name>
      <initial_velocity>0.0</initial_velocity>
      <topic>right_motor_rpm</topic>
      <p_gain>0.5</p_gain>
      <i_gain>0.1</i_gain>
  </plugin>
</gazebo>
```

**اضافه کنید نویز به IMU**:
```xml
<gazebo reference="zed_imu_link">
    <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
        <update_rate>100</update_rate>
        <visualize>true</visualize>
        <topic>/zed/zed_node/imu/data_raw</topic>
        <imu>
            <angular_velocity>
                <x><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></x>
                <y><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></y>
                <z><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></z>
            </angular_velocity>
            <linear_acceleration>
                <x><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></x>
                <y><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></y>
                <z><noise type="gaussian"><mean>0.0</mean><stddev>0.05</stddev></noise></z>
            </linear_acceleration>
            <orientation>
                <x><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></x>
                <y><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></y>
                <z><noise type="gaussian"><mean>0.0</mean><stddev>0.01</stddev></noise></z>
            </orientation>
        </imu>
    </sensor>
</gazebo>
```

### مرحله 6: نصب Dependencies

```bash
cd ~/robotic_course-main
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### مرحله 7: Build کردن پروژه

```bash
cd ~/robotic_course-main
colcon build --symlink-install
source install/setup.bash
```

برای source خودکار در هر ترمینال:
```bash
echo "source ~/robotic_course-main/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## ▶️ اجرای پروژه

### راه‌اندازی کامل (6 ترمینال)

#### ترمینال 1: Gazebo + RViz + RTABMap

```bash
source ~/robotic_course-main/install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

**منتظر بمانید تا Gazebo کاملاً لود شود**

#### ترمینال 2: کنترلر CMD_VEL به RPM

```bash
source ~/robotic_course-main/install/setup.bash
ros2 run robot_description cmd_vel_controller_node
```

#### ترمینال 3: Prediction Node

```bash
source ~/robotic_course-main/install/setup.bash
ros2 run robot_local_localization prediction_node
```

#### ترمینال 4: Measurement Node

```bash
source ~/robotic_course-main/install/setup.bash
ros2 run robot_local_localization measurement_node
```

#### ترمینال 5: EKF Node

```bash
source ~/robotic_course-main/install/setup.bash
ros2 run robot_local_localization ekf_node
```

#### ترمینال 6: Test Node (مسیر مستطیلی)

```bash
source ~/robotic_course-main/install/setup.bash
ros2 run robot_local_localization test_node \
  --ros-args \
  -p rectangle_length:=2.0 \
  -p rectangle_width:=1.0 \
  -p linear_speed:=0.2 \
  -p angular_speed:=0.3
```

---

## 📊 نمایش نتایج در RViz

### اضافه کردن مسیرها

در پنجره RViz که با Gazebo باز شده:

1. کلیک روی دکمه **Add** (پایین سمت چپ)
2. انتخاب **By topic** → **Path**
3. مسیرهای زیر را اضافه کنید:

| Topic | رنگ | توضیحات |
|-------|-----|---------|
| `/paths/ground_truth` | آبی | مرجع واقعی (از encoders) |
| `/paths/ekf` | سبز | تخمین EKF (بهترین) |
| `/paths/motion_model` | زرد | فقط پیش‌بینی (drift دارد) |
| `/paths/measurement` | قرمز | فقط اندازه‌گیری |

### تنظیمات نمایش

برای هر Path:
- **Line Width**: 0.05
- **Alpha**: 1.0
- **Pose Style**: None

---

## 🎥 ضبط داده (Bonus)

### ترمینال 7: Rosbag

```bash
cd ~/robotic_course-main
mkdir -p rosbags
cd rosbags

ros2 bag record -o ekf_rectangular_test \
  /ekf/odom \
  /wheel_encoder/odom \
  /motion_model/odom \
  /measurement_model \
  /paths/ekf \
  /paths/ground_truth \
  /paths/motion_model \
  /paths/measurement \
  /cmd_vel \
  /zed/zed_node/imu/data_raw \
  /vo/odom \
  /tf \
  /tf_static
```

### نمایش با Foxglove Studio

```bash
# نصب Foxglove
sudo snap install foxglove-studio

# باز کردن
foxglove-studio
```

در Foxglove:
1. **Open local file**
2. انتخاب فایل `.db3` از پوشه `rosbags/ekf_rectangular_test/`
3. اضافه کردن پنل‌های:

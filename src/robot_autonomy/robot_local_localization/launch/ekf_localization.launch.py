#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Prediction Node (Motion Model)
        Node(
            package='robot_local_localization',
            executable='prediction_node',
            name='prediction_node',
            output='screen',
            parameters=[{
		'use_sim_time': True,
                'wheel_separation': 0.45,
                'wheel_radius': 0.1,
                'cmd_vel_topic': '/cmd_vel',
                'motion_model_topic': '/motion_model/odom',
                'frame_id': 'odom',
                'child_frame_id': 'base_link'
            }]
        ),
        
        # Measurement Node (Sensor Fusion)
        Node(
            package='robot_local_localization',
            executable='measurement_node',
            name='measurement_node',
            output='screen',
            parameters=[{
		'use_sim_time': True,
                'imu_topic': '/zed/zed_node/imu/data_raw',
                'vo_topic': '/vo/odom',
                'measurement_topic': '/measurement_model/pose',
                'frame_id': 'odom'
            }]
        ),
        
        # EKF Node
        Node(
            package='robot_local_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[{
		'use_sim_time': True,
                'motion_model_topic': '/motion_model/odom',
                'measurement_topic': '/measurement_model/pose',
                'ekf_odom_topic': '/ekf/odom',
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'publish_tf': True
            }]
        ),
        
        # Test Node (Rectangular Trajectory)
        Node(
            package='robot_local_localization',
            executable='test_node',
            name='test_node',
            output='screen',
            parameters=[{
		'use_sim_time': True,
                'ekf_odom_topic': '/ekf/odom',
                'motion_model_topic': '/motion_model/odom',
                'measurement_topic': '/measurement_model/pose',
                'wheel_odom_topic': '/wheel_encoder/odom',
                'cmd_vel_topic': '/cmd_vel',
                'linear_velocity': 0.3,
                'angular_velocity': 0.5,
                'rectangle_width': 4.0,
                'rectangle_height': 2.0
            }]
        ),
    ])

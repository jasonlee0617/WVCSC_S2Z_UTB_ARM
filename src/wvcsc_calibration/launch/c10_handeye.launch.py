"""Standalone Alicia-M/C10 eye-in-hand calibration session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('wvcsc_calibration')
    bringup_share = get_package_share_directory('wvcsc_bringup')
    c10_share = get_package_share_directory('wvcsc_c10_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/v4l/by-id/usb-Synria_C10-video-index0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/c10_intrinsics.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                c10_share, 'launch', 'c10_camera.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items()),
        # 手眼标定只需要机械臂、C10 和其 TF；不加载完整传感器启动链，
        # 因而不会启动底盘、LiDAR、IMU、EKF 或 Nav2。
        Node(
            package='wvcsc_arm_task', executable='motion_control',
            parameters=[{
                'base_frame': 'alicia_base_link',
                'group_name': 'arm',
                'tool_link': 'tool0',
                'planning_pipeline_id': 'ompl',
                'planner_id': 'RRTConnectFast',
                'velocity_scaling': 0.1,
                'acceleration_scaling': 0.1,
                'planning_time': 5.0,
                'execution_timeout': 90.0,
                'use_sim_time': False,
            }],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, 'launch', 'real_arm.launch.py')),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'use_rviz': 'true',
                'publish_robot_state': 'true',
            }.items()),
        Node(
            package='ros2_aruco', executable='aruco_node',
            parameters=[os.path.join(
                calibration_share, 'config', 'aruco_c10.yaml')],
            output='screen'),
        Node(
            package='wvcsc_calibration', executable='marker_tf',
            parameters=[{
                'tracking_base_frame': 'camera_color_optical_frame',
                'tracking_marker_frame': 'calibration_aruco',
                'marker_id': 1,
                # A sample is taken only after arm settling; average a short
                # stationary window to reduce planar-PnP quantization.
                'smoothing_window': 15,
            }],
            output='screen'),
        # 自动采集只需要服务端；不启动会与 s/q 流程重复的 RQt 手工界面。
        Node(
            package='easy_handeye2', executable='handeye_server',
            name='handeye_server',
            parameters=[{
                'name': 'wvcsc_c10',
                'calibration_type': 'eye_in_hand',
                'robot_base_frame': 'alicia_base_link',
                'robot_effector_frame': 'tool0',
                'tracking_base_frame': 'camera_color_optical_frame',
                'tracking_marker_frame': 'calibration_aruco',
                'use_sim_time': False,
            }],
            output='screen'),
    ])

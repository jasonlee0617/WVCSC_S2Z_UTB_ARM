# 中文说明：交互式 C10 手眼标定启动入口。
# 采样需要操作员在独立终端确认，launch 子进程不得假设 stdin 是可交互 TTY。
"""Alicia-M + C10 eye-in-hand calibration session (interactive sampling).

The arm model reuses the Alicia-M native URDF (no vehicle chassis), and the
C10 camera chain (tool0 → camera_link → camera_color_optical_frame) is
published via static TF because it is not part of the arm-only URDF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('wvcsc_calibration')
    alicia_bringup_share = get_package_share_directory('alicia_m_bringup')
    c10_share = get_package_share_directory('wvcsc_c10_camera')

    return LaunchDescription([
        # ── C10 camera ──────────────────────────────────────────
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video2'),
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

        # ── Identity TF alias: base_link → alicia_base_link ─────
        # moveit_hardware.launch.py uses Alicia URDF root frame "base_link",
        # but easy_handeye2 and downstream tasks use "alicia_base_link".
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_link', 'alicia_base_link']),

        # ── Static TF: tool0 → camera_link (temporary extrinsics) ──
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=[
                '-0.055', '0', '-0.10',
                '0', '-1.57079632679', '0',
                'tool0', 'camera_link',
            ]),
        # ── Static TF: camera_link → camera_color_optical_frame (REP-103) ──
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=[
                '0', '0', '0',
                '-1.57079632679', '0', '-1.57079632679',
                'camera_link', 'camera_color_optical_frame',
            ]),

        # ── Alicia-M arm (MoveIt + ros2_control, arm-only URDF) ──
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                alicia_bringup_share, 'launch', 'moveit_hardware.launch.py')),
            launch_arguments={
                'serial_port': LaunchConfiguration('serial_port'),
                'use_rviz': 'true',
            }.items()),

        # ── ArUco detection ─────────────────────────────────────
        Node(
            package='ros2_aruco', executable='aruco_node',
            parameters=[os.path.join(
                calibration_share, 'config', 'aruco_c10.yaml')],
            output='both'),

        # ── ArUco TF publisher (publishes calibration_aruco frame) ──
        Node(
            package='wvcsc_calibration', executable='marker_tf',
            parameters=[{
                'tracking_base_frame': 'camera_color_optical_frame',
                'tracking_marker_frame': 'calibration_aruco',
                'marker_id': 1,
                'smoothing_window': 15,
            }],
            output='both'),

        # ── ArUco visualization overlay on C10 image ────────────
        Node(
            package='wvcsc_calibration', executable='visualize_aruco_marker',
            name='wvcsc_aruco_overlay',
            parameters=[{
                'marker_size_m': 0.070,
                'aruco_dictionary_id': 'DICT_5X5_250',
                'marker_id': 1,
                'image_topic': '/camera/color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'output_topic': '/calibration/aruco_debug_image',
                'use_sim_time': False,
            }],
            output='both'),

        # ── easy_handeye2 interactive calibration GUI ──────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('easy_handeye2'),
                'launch', 'calibrate.launch.py')),
            launch_arguments={
                'name': 'wvcsc_c10',
                'calibration_type': 'eye_in_hand',
                'robot_base_frame': 'alicia_base_link',
                'robot_effector_frame': 'tool0',
                'tracking_base_frame': 'camera_color_optical_frame',
                'tracking_marker_frame': 'calibration_aruco',
            }.items()),
    ])

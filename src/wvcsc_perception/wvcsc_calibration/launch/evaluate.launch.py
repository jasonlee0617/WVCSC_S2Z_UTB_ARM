"""Alicia-M + C10 eye-in-hand verification (loads saved calibration).

The arm model reuses the Alicia-M native URDF (no vehicle chassis).
handeye_publisher publishes the calibrated tool0→camera_color_optical_frame
transform from the saved calibration result, allowing RViz visual verification.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wvcsc_bringup.handeye_calibration_paths import resolve_handeye_calibration


def _evaluate_actions(context):
    value = str(resolve_handeye_calibration(
        LaunchConfiguration('handeye_calibration').perform(context)))
    return [
        Node(
            package='easy_handeye2', executable='handeye_publisher',
            name='handeye_publisher',
            parameters=[{
                'name': 'wvcsc_c10',
                'calibration_file': value,
                'calibration_type': 'eye_in_hand',
                'publish_rate_hz': 10.0,
            }],
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('easy_handeye2'),
                'launch', 'evaluate.launch.py')),
            launch_arguments={
                'name': 'wvcsc_c10',
                'calibration_file': value,
            }.items()),
    ]


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

        # ── ArUco visualization overlay ─────────────────────────
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

        # ── ArUco TF publisher ──────────────────────────────────
        Node(
            package='wvcsc_calibration', executable='marker_tf',
            parameters=[{
                'tracking_base_frame': 'camera_color_optical_frame',
                'tracking_marker_frame': 'calibration_aruco',
                'marker_id': 1,
                'smoothing_window': 15,
            }],
            output='both'),

        DeclareLaunchArgument('handeye_calibration', default_value='latest_real'),
        # ── Hand-eye TF publisher + evaluate GUI ────────────────
        OpaqueFunction(function=_evaluate_actions),
    ])

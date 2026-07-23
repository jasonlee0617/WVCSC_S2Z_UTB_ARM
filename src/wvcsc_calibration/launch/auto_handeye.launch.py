"""Start one operator-confirmed Alicia-M/C10 hand-eye calibration session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    calibration_share = get_package_share_directory('wvcsc_calibration')
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/c10_intrinsics.yaml')),
        # Keep this entry deliberately thin.  c10_handeye.launch.py is the
        # single owner of C10, Alicia-M, ArUco, marker TF and easy_handeye2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                calibration_share, 'launch', 'c10_handeye.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'serial_port': LaunchConfiguration('serial_port'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items()),
        Node(
            package='wvcsc_calibration', executable='auto_calibration_collector',
            name='auto_calibration_collector',
            parameters=[
                os.path.join(calibration_share, 'config',
                             'auto_handeye_alicia.yaml'),
                {'auto_start': False, 'use_sim_time': False},
            ],
            output='both'),
    ])

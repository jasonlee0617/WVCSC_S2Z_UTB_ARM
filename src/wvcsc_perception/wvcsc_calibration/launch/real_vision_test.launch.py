"""Compatibility wrapper for the canonical RGB-vision test launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vision_share = get_package_share_directory('wvcsc_rgb_vision')
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url', default_value=(
                'package://wvcsc_c10_camera/config/c10_intrinsics.yaml')),
        DeclareLaunchArgument(
            'yolo_python_executable', default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python')),
        DeclareLaunchArgument('inference_mode', default_value='fruits'),
        DeclareLaunchArgument('publish_visualization', default_value='true'),
        DeclareLaunchArgument('standalone_mode', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                vision_share, 'launch', 'real_vision_test.launch.py')),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name in (
                    'video_device', 'camera_info_url',
                    'yolo_python_executable', 'inference_mode',
                    'publish_visualization', 'standalone_mode')
            }.items()),
    ])

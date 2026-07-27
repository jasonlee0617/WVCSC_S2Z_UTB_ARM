"""C10 8x6, 25 mm checkerboard intrinsic calibration session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wvcsc_c10_camera')
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'c10_camera.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
            }.items()),
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            name='c10_intrinsic_calibrator',
            arguments=[
                '--size', '8x6',
                '--square', '0.025',
                '--ros-args',
                '-r', 'image:=/camera/color/image_raw',
                '-r', 'camera:=/camera/color',
            ],
            output='screen'),
    ])

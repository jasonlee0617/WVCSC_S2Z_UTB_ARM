"""Standalone real-camera YOLO test: C10 camera plus both YOLO stages."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_share = get_package_share_directory('wvcsc_c10_camera')
    vision_share = get_package_share_directory('wvcsc_rgb_vision')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='C10 V4L2 device (current JR0037 index0).'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/c10_intrinsics.yaml')),
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python'),
            description='Python interpreter containing the YOLO runtime.'),
        DeclareLaunchArgument(
            'inference_mode', default_value='fruits',
            description='YOLO mode: tree, fruits, or target.'),
        DeclareLaunchArgument(
            'publish_visualization', default_value='true'),
        DeclareLaunchArgument(
            'standalone_mode', default_value='true',
            description='Process frames without /mission/status or a robot.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                camera_share, 'launch', 'c10_camera.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items()),

        Node(
            package='wvcsc_rgb_vision', executable='two_stage_yolo',
            name='wvcsc_two_stage_yolo',
            prefix=[LaunchConfiguration('yolo_python_executable')],
            additional_env={
                'PYTHONNOUSERSITE': '1',
                'YOLO_CONFIG_DIR': '/tmp/wvcsc_ultralytics',
            },
            parameters=[
                os.path.join(vision_share, 'config', 'vision_real.yaml'),
                {
                    'use_sim_time': False,
                    'standalone_mode': ParameterValue(
                        LaunchConfiguration('standalone_mode'),
                        value_type=bool),
                    'inference_mode': LaunchConfiguration('inference_mode'),
                    'publish_visualization': ParameterValue(
                        LaunchConfiguration('publish_visualization'),
                        value_type=bool),
                },
            ],
            output='screen'),
    ])

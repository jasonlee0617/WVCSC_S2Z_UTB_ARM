"""Real C10 camera + YOLO inference test (no arm, no navigation)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    c10_share = get_package_share_directory('wvcsc_c10_camera')
    vision_share = get_package_share_directory('wvcsc_rgb_vision')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/v4l/by-id/usb-Synria_C10-video-index0'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/c10_intrinsics.yaml')),
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python')),

        # ── C10 camera ──────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                c10_share, 'launch', 'c10_camera.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items()),

        # ── YOLO two-stage inference (tree detect + fruit seg) ──
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
                    'inference_mode': 'fruits',
                },
            ],
            output='screen'),
    ])

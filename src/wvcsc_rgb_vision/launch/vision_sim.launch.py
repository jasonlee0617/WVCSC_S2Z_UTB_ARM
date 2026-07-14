import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wvcsc_rgb_vision'),
        'config',
        'vision_sim.yaml',
    )
    return LaunchDescription([
        Node(
            package='wvcsc_rgb_vision', executable='alignment_gate',
            parameters=[config], output='screen'),
        Node(
            package='wvcsc_rgb_vision', executable='mock_vision',
            parameters=[config], output='screen'),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('controller_pkg'), 'config', 'fault.ini')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        Node(
            package='controller_pkg',
            executable='relay_controller',
            name='relay_controller',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
            }],
            output='screen',
        ),
    ])

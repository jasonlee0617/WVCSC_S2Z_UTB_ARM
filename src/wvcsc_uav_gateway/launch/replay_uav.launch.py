import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wvcsc_uav_gateway')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(share, 'config', 'replay_targets.yaml')),
        Node(
            package='wvcsc_uav_gateway',
            executable='replay_uav_gateway',
            parameters=[{'config_file': LaunchConfiguration('config_file')}],
            output='screen',
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('wvcsc_mission_manager')
    auto_start = LaunchConfiguration('auto_start')
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='false'),
        Node(
            package='wvcsc_mission_manager',
            executable='mission_manager',
            parameters=[
                os.path.join(share, 'config', 'mission_manager.yaml'),
                {'auto_start': ParameterValue(auto_start, value_type=bool)},
            ],
            output='screen'),
    ])

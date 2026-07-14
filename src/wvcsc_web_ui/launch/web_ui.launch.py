import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('wvcsc_web_ui')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='8080'),
        Node(
            package='wvcsc_web_ui',
            executable='web_server',
            parameters=[
                os.path.join(share, 'config', 'web_ui.yaml'),
                {
                    'host': LaunchConfiguration('host'),
                    'port': ParameterValue(
                        LaunchConfiguration('port'), value_type=int),
                },
            ],
            output='screen',
        ),
    ])

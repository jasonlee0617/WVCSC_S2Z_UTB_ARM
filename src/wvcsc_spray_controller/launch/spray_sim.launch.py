import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wvcsc_spray_controller')
    return LaunchDescription([
        Node(
            package='wvcsc_spray_controller',
            executable='spray_simulator',
            parameters=[os.path.join(share, 'config', 'spray_sim.yaml')],
            output='screen',
        ),
    ])

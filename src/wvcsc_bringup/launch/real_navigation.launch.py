"""Localization-only real navigation: map server + AMCL + Nav2.

The mapping stack is deliberately absent from this launch file.  Nav2 writes to
``/cmd_vel_nav``; only ``wvcsc_safety`` may forward commands to the chassis on
``/cmd_vel``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    bringup_share = get_package_share_directory('wvcsc_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    navigation_share = get_package_share_directory('my_navigation2')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # The two remaps are scoped to Nav2.  The base driver still subscribes to
    # /cmd_vel, which is owned exclusively by the safety gate.
    nav2 = GroupAction(actions=[
        SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        SetRemap(src='/odom', dst='/ekf_odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'namespace': '',
                'use_namespace': 'False',
                'slam': 'False',
                'map': map_file,
                'use_sim_time': 'False',
                'params_file': params_file,
                'autostart': 'True',
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'info',
            }.items(),
        ),
    ])

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'False',
            'use_sim_time': 'False',
        }.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(navigation_share, 'maps', 'map_new.yaml'),
            description='Absolute map YAML used by map_server and AMCL.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_share, 'config', 'real', 'nav2_corn.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        nav2,
        rviz,
    ])

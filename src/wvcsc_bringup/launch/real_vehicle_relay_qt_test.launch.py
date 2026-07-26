"""Qt-selected route validation with real vehicle/relay and a fake arm."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wvcsc_bringup.path_defaults import latest_map_yaml


def generate_launch_description():
    bringup_share = get_package_share_directory('wvcsc_bringup')
    controller_share = get_package_share_directory('controller_pkg')
    mission_share = get_package_share_directory('wvcsc_mission_manager')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'real_navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'start_vehicle_stack': 'true',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    relay_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            controller_share, 'launch', 'controller.launch.py')),
        launch_arguments={
            'config_file': LaunchConfiguration('relay_config_file'),
        }.items(),
    )
    mission_manager = Node(
        package='wvcsc_mission_manager', executable='mission_manager',
        parameters=[
            os.path.join(mission_share, 'config', 'mission_manager.yaml'),
            {
                'use_sim_time': False,
                'arm_base_yaw_rad': math.pi,
                'wide_relay_channel': 1,
                'arm_relay_channel': 2,
                'relay_service_name': '/relay/set',
            },
        ],
        remappings=[('/odom', '/ekf_odom')],
        output='screen',
    )
    qt_editor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'nav2_qt.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_qt_gui')),
        launch_arguments={
            'use_sim_time': 'false',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'goal_pose_topic': '/manual_goal_pose',
        }.items(),
    )
    fake_arm = Node(
        package='wvcsc_bringup',
        executable='fake_arm_spray_action.py',
        name='fake_arm_spray_action',
        parameters=[{
            'relay_service_name': '/relay/set',
            'relay_channel': 2,
            'relay_service_timeout_sec': 2.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=latest_map_yaml()),
        DeclareLaunchArgument(
            'relay_config_file',
            default_value=os.path.join(
                controller_share, 'config', 'fault.ini')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_qt_gui', default_value='true'),
        navigation,
        relay_controller,
        mission_manager,
        fake_arm,
        qt_editor,
    ])

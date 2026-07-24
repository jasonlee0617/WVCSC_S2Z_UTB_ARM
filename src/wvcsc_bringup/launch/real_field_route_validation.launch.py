"""Real Nav2 + relay integration test with a simulated arm action.

This entry point intentionally omits Alicia-M, C10, YOLO, MoveIt and Servo.
It drives the vehicle and physical relay controller, while the fake arm Action
server pulses relay channel 2 and returns successful results at inspect stops.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('wvcsc_bringup')
    controller_share = get_package_share_directory('controller_pkg')

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
    route_manager = Node(
        package='wvcsc_bringup',
        executable='field_route_manager.py',
        name='wvcsc_field_route_manager',
        parameters=[{
            'use_sim_time': False,
            'mission_file': LaunchConfiguration('mission_file'),
            'map_file': LaunchConfiguration('map'),
            'auto_start': True,
            'wide_relay_channel': 1,
            'arm_relay_channel': 2,
        }],
        remappings=[('/odom', '/ekf_odom')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_file',
            default_value=os.path.expanduser(
                '~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/real/'
                'field_route_corn.yaml')),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser(
                '~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml')),
        DeclareLaunchArgument(
            'relay_config_file',
            default_value=os.path.join(
                controller_share, 'config', 'fault.ini')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        navigation,
        relay_controller,
        fake_arm,
        route_manager,
    ])

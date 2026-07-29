"""Standalone real mapping launch matching my_cartographer's proven chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('wvcsc_bringup')
    cartographer_share = get_package_share_directory('my_cartographer')
    vehicle_share = get_package_share_directory('wtb_car_driver')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = LaunchConfiguration('cartographer_config_dir')
    basename = LaunchConfiguration('configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period = LaunchConfiguration('publish_period_sec')

    # This include already owns CAN, chassis, LiDAR, pointcloud conversion,
    # IMU, EKF and robot state publishers. The parent launch therefore starts no
    # other hardware launch in mapping mode.
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            vehicle_share, 'launch', 'start_wtb_car_fdimu.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'open_rviz': 'false',
            'enable_pointcloud_to_laserscan': 'true',
            'enable_ackermann': 'true',
        }.items(),
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', basename,
        ],
        remappings=[('/odom', '/ekf_odom')],
        output='screen',
    )
    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period,
        ],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            bringup_share, 'rviz', 'real_cartographer.rviz')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=os.path.join(cartographer_share, 'config')),
        DeclareLaunchArgument(
            'configuration_basename', default_value='cartographer.lua'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='0.5'),
        hardware,
        cartographer,
        occupancy_grid,
        rviz,
    ])

# 中文说明：实机独立导航入口，启动车辆传感器、定位、Nav2 与导航 RViz。
# Qt 完整任务会复用其中的导航链，但任务点加载和喷洒触发由 MissionManager 负责。
# 地图路径、目标话题和底盘控制接口是外部可见契约，不能在此处静默改名。
"""Standalone real navigation aligned with wtb_navigation2_fdimu.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from wvcsc_bringup.path_defaults import latest_map_yaml


def generate_launch_description():
    bringup_share = get_package_share_directory('wvcsc_bringup')
    navigation_share = get_package_share_directory('my_navigation2')
    nav2_share = get_package_share_directory('nav2_bringup')
    vehicle_share = get_package_share_directory('wtb_car_driver')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_goal_topic = LaunchConfiguration('rviz_goal_topic')
    nav_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
        },
        convert_types=True,
    )

    # This is the field-validated vehicle, LiDAR, IMU, and EKF chain. It does
    # not start C10. The full mission provides its own unified sensor stack.
    vehicle_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            vehicle_share, 'launch', 'start_wtb_car_fdimu.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_vehicle_stack')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'open_rviz': 'false',
            'enable_pointcloud_to_laserscan': 'true',
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav_params,
            'tf_buffer_size': '300',
        }.items(),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='real_navigation_rviz',
        arguments=['-d', os.path.join(
            bringup_share, 'rviz', 'real_navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        # The shared RViz config publishes its 2D Goal on
        # /manual_goal_pose.  Standalone navigation remaps it to Nav2's
        # /goal_pose; the full Qt mission overrides this back to the manual
        # recorder topic.
        remappings=[('/manual_goal_pose', rviz_goal_topic)],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_vehicle_stack', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_goal_topic',
            default_value='/goal_pose',
            description=(
                'Destination for RViz 2D Goal Pose. Standalone navigation '
                'defaults to Nav2; the full Qt mission keeps manual capture.')),
        DeclareLaunchArgument(
            'map',
            default_value=latest_map_yaml(),
            description='Absolute map YAML used by map_server and AMCL.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                navigation_share, 'param', 'wtb_nav2_params.yaml')),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=os.path.join(
                navigation_share, 'behavior_trees',
                'navigate_to_pose_ackermann.xml'),
            description=(
                'NavigateToPose recovery tree for the Ackermann vehicle. '
                'It intentionally has no in-place Spin recovery.')),
        vehicle_stack,
        nav2,
        rviz,
    ])

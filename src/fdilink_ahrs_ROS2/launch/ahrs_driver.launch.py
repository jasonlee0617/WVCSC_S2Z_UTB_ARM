import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

     # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('fdilink_ahrs'),  # 替换为你的功能包名称
        'config',
        'ahrs_params.yaml'
    )

    ahrs_driver = Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        name="ahrs_driver_node",  # 显式指定节点名称，需与yaml中一致
        parameters=[config_dir],  # 加载配置文件
        output="screen"
    )

    launch_description =LaunchDescription()
    launch_description.add_action(ahrs_driver)
    #该节点将IMU传感器的姿态信息转换为ROS2的TF变换系统
#    launch_description.add_action(imu_tf)
    return launch_description

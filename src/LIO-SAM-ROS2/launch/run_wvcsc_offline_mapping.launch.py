import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')

    params_file = LaunchConfiguration('params_file')
    launch_rviz = LaunchConfiguration('launch_rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'wvcsc_mapping_params.yaml'),
        description='Path to the WVCSC LIO-SAM mapping parameter file.'
    )
    launch_rviz_declare = DeclareLaunchArgument('launch_rviz', default_value='false')

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 1.0 map odom'.split(' '),
        parameters=[params_file],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share_dir, 'config', 'rviz2.rviz')],
        parameters=[params_file],
        output='screen',
        condition=IfCondition(launch_rviz)
    )

    return LaunchDescription([
        params_declare,
        launch_rviz_declare,
        static_map_to_odom,
        Node(package='lio_sam', executable='lio_sam_imuPreintegration', name='lio_sam_imuPreintegration', parameters=[params_file], output='screen'),
        Node(package='lio_sam', executable='lio_sam_imageProjection', name='lio_sam_imageProjection', parameters=[params_file], output='screen'),
        Node(package='lio_sam', executable='lio_sam_featureExtraction', name='lio_sam_featureExtraction', parameters=[params_file], output='screen'),
        Node(package='lio_sam', executable='lio_sam_mapOptimization', name='lio_sam_mapOptimization', parameters=[params_file], output='screen'),
        rviz,
    ])

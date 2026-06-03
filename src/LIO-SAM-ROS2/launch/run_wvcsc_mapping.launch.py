import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('lio_sam')
    bringup_dir = get_package_share_directory('wvcsc_autoware_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_hardware = LaunchConfiguration('launch_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'wvcsc_mapping_params.yaml'),
        description='Path to the WVCSC LIO-SAM mapping parameter file.'
    )
    use_sim_time_declare = DeclareLaunchArgument('use_sim_time', default_value='false')
    launch_hardware_declare = DeclareLaunchArgument('launch_hardware', default_value='true')
    launch_rviz_declare = DeclareLaunchArgument('launch_rviz', default_value='false')

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'hardware.launch.xml')),
        condition=IfCondition(launch_hardware),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

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
        use_sim_time_declare,
        launch_hardware_declare,
        launch_rviz_declare,
        hardware,
        static_map_to_odom,
        Node(package='lio_sam', executable='lio_sam_imuPreintegration', name='lio_sam_imuPreintegration', parameters=[params_file], output='screen'),
        Node(package='lio_sam', executable='lio_sam_imageProjection', name='lio_sam_imageProjection', parameters=[params_file], output='screen'),
        Node(package='lio_sam', executable='lio_sam_featureExtraction', name='lio_sam_featureExtraction', parameters=[params_file], output='screen'),
        Node(package='lio_sam', executable='lio_sam_mapOptimization', name='lio_sam_mapOptimization', parameters=[params_file], output='screen'),
        rviz,
    ])

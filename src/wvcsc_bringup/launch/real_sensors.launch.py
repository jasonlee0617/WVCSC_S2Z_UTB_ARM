"""Real chassis, LiDAR, IMU, EKF, C10 and the unified TF publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    c10_share = get_package_share_directory('wvcsc_c10_camera')
    description_share = get_package_share_directory('wvcsc_description')
    lidar_share = get_package_share_directory('lslidar_driver')
    # 旧版 FDI IMU 已停用，仅保留注释用于回滚：
    # imu_share = get_package_share_directory('fdilink_ahrs')
    # imu = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         imu_share, 'launch', 'ahrs_driver.launch.py')),
    # )

    # 当前实车 IMU 使用 yesense_std_ros2，向下游保持 /imu 接口不变。
    yesense_share = get_package_share_directory('yesense_std_ros2')
    vehicle_share = get_package_share_directory('wtb_car_driver')

    xacro_file = os.path.join(
        description_share, 'urdf', 'wvcsc_utb_alicia.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' alicia_base_link:=alicia_base_link',
            ' use_collision_meshes:=true',
            ' enable_arm_control:=true',
            ' enable_ackermann:=true',
            ' enable_gazebo_ros2_control:=false',
            ' enable_c10_camera:=true',
            ' enable_c10_gazebo:=false',
            ' ros2_control_plugin:=alicia_m_driver/AliciaHardwareInterface',
            ' serial_port:=', LaunchConfiguration('serial_port'),
            ' baudrate:=', LaunchConfiguration('baudrate'),
            ' control_mode:=', LaunchConfiguration('control_mode'),
            ' default_speed:=', LaunchConfiguration('default_speed'),
            # Keep vector-valued xacro arguments as one quoted shell token.
            # Calibrated values contain negative components; without quotes
            # xacro interprets e.g. ``-0.021`` as a command-line option.
            ' c10_mount_xyz:="', LaunchConfiguration('c10_mount_xyz'), '"',
            ' c10_mount_rpy:="', LaunchConfiguration('c10_mount_rpy'), '"',
            ' nozzle_mount_xyz:="', LaunchConfiguration('nozzle_mount_xyz'), '"',
            ' nozzle_mount_rpy:="', LaunchConfiguration('nozzle_mount_rpy'), '"',
        ]), value_type=str),
    }

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            c10_share, 'launch', 'c10_camera.launch.py')),
        launch_arguments={
            'video_device': LaunchConfiguration('c10_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }.items(),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            lidar_share, 'launch', 'lslidar_cx_launch.py')),
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            yesense_share, 'launch', 'yesense_node.launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'c10_device',
            default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/'
                'c10_intrinsics.yaml')),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('control_mode', default_value='pv'),
        DeclareLaunchArgument('default_speed', default_value='0.5'),
        DeclareLaunchArgument('c10_mount_xyz', default_value='-0.055 0 -0.10'),
        DeclareLaunchArgument(
            'c10_mount_rpy', default_value='0 -1.57079632679 0'),
        DeclareLaunchArgument('nozzle_mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('nozzle_mount_rpy', default_value='0 0 0'),
        Node(
            package='can_bridge', executable='can_bridge_node',
            name='can_bridge_node', output='screen'),
        Node(
            package='wtb_car_driver', executable='wtb_car', name='wtb_car',
            parameters=[{
                'WHEELBASE': 0.82,
                'vel_scale': 1.0,
                'steer_offset': 0.0,
                'min_speed': 0.0005,
                'use_sim_time': False,
            }],
            # Nav2 uses /cmd_vel. Disable the driver's secondary ingress so
            # stale TwistStamped commands cannot compete with navigation.
            remappings=[
                ('/twist_cmd', '/wvcsc_bringup/disabled_twist_cmd')],
            output='screen'),
        lidar,
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/point_cloud_raw'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'laser',
                'min_height': -0.75,
                'max_height': 0.5,
                'transform_tolerance': 1.0,
                'angle_min': -3.1415926,
                'angle_max': 3.1415926,
                'angle_increment': 0.0003,
                'scan_time': 0.3333,
                'range_min': 0.5,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 50.0,
                'use_sim_time': False,
            }],
            output='screen'),
        imu,
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(
                vehicle_share, 'config', 'ekf_wtb_fdimu.yaml')],
            remappings=[('/odometry/filtered', '/ekf_odom')],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': False}],
            output='screen'),
        camera,
    ])

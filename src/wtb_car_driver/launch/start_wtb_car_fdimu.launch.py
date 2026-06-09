from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import subprocess

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='False') 
    enable_pointcloud_to_laserscan = LaunchConfiguration('enable_pointcloud_to_laserscan')

    # 创建启动描述
    ld = LaunchDescription()

    open_rviz_arg = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Whether to open RViz'
    )

    enable_pointcloud_to_laserscan_arg = DeclareLaunchArgument(
        'enable_pointcloud_to_laserscan',
        default_value='true',
        description='Whether to run pointcloud_to_laserscan'
    )
    
    # 找到待包含的 launch 文件
    can_bridge_launch = os.path.join(
        get_package_share_directory('can_bridge'),
        'launch',
        'can_bridge.launch.py'
    )

    can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(can_bridge_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
        )

    
    

    # 关节状态发布器
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 获取xacro文件路径
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('wtb_car_driver'),
            'urdf',
            'wtb_car.xacro'
        ])
    ])
    
    # 声明robot_description参数
    robot_description = {'robot_description': robot_description_content}

    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, 
                    {'use_sim_time': use_sim_time},
                  
                    ]
    )


    # 底盘驱动
    wtb_car = Node(
        package='wtb_car_driver',
        executable='wtb_car',
        name='wtb_car',
        output='screen',
         parameters=[
            # {'WHEELBASE': 0.66},
            {'WHEELBASE': 0.82},
            {'vel_scale': 1.0},
            #  {'vel_scale': 2.0},
            {'steer_offset': 0.0},
            {'min_speed': 0.0005},
             {'use_sim_time': use_sim_time}
        ]
    )
    
  

    Lslidar_dir = get_package_share_directory('lslidar_driver')
    Lslidar_launch_dir = os.path.join(Lslidar_dir, 'launch')
    lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Lslidar_launch_dir, 'lslidar_cx_launch.py')),)

    # 3D转2D点云
    pointcloud_to_laserscan = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                         ('cloud_in', '/point_cloud_raw'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'laser',
                'min_height': -0.75,
                'max_height': 0.5,
                'transform_tolerance': 1.0,
                'angle_min': -3.1415926,
                'angle_max': 3.1415926,
                'angle_increment': 0.0003,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.5,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 50.0,
                'use_sim_time': use_sim_time,
            }],
            name='pointcloud_to_laserscan'
    )

    # IMU节点
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('fdilink_ahrs'),
                'launch',
                'ahrs_driver.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('wtb_car_driver'),
                'config',
                'ekf_wtb_fdimu.yaml'
            ])
        ],
        remappings=[
            ('/odometry/filtered', '/ekf_odom')
        ]
    )

    
    # RViz
    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration('open_rviz')),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                arguments=[
                    '-d',
                    PathJoinSubstitution([
                        FindPackageShare('wtb_car_driver'),
                        'rviz',
                        '1.rviz'
                    ])
                ],
                output='screen'
            )
        ]
    )
    
    
    # 添加参数声明
    ld.add_action(open_rviz_arg)
    ld.add_action(enable_pointcloud_to_laserscan_arg)

    # 添加节点
    ld.add_action(can_launch)
    ld.add_action(imu_launch)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(wtb_car)
    ld.add_action(lidar_launch)
    ld.add_action(
        GroupAction(
            condition=IfCondition(enable_pointcloud_to_laserscan),
            actions=[pointcloud_to_laserscan]
        )
    )
    ld.add_action(ekf_node)
    ld.add_action(rviz)
    
    return ld

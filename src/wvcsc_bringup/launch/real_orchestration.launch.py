"""Real perception, arm task and fixed five-point field orchestration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wvcsc_bringup.path_defaults import latest_field_route, latest_map_yaml


def generate_launch_description():
    bringup_share = get_package_share_directory('wvcsc_bringup')
    description_share = get_package_share_directory('wvcsc_description')
    vision_share = get_package_share_directory('wvcsc_rgb_vision')
    controller_share = get_package_share_directory('controller_pkg')
    real_config = os.path.join(bringup_share, 'config', 'real')

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
    arm_motion_parameters = {
        'base_frame': 'alicia_base_link',
        'group_name': 'arm',
        'tool_link': 'tool0',
        'planning_pipeline_id': 'ompl',
        'planner_id': 'RRTConnectFast',
        'velocity_scaling': ParameterValue(
            LaunchConfiguration('arm_velocity_scaling'), value_type=float),
        'acceleration_scaling': ParameterValue(
            LaunchConfiguration('arm_acceleration_scaling'), value_type=float),
        'retime_service_name': '/retime_trajectory',
        'retime_timeout': 5.0,
        'execution_timeout': 90.0,
        'planning_time': 5.0,
        'gripper_action': '/gripper_controller/gripper_cmd',
        'gripper_open_position': 0.0,
        'gripper_closed_position': -0.05,
        'gripper_max_effort': 5.0,
        'use_sim_time': False,
    }

    yolo = Node(
        package='wvcsc_rgb_vision', executable='two_stage_yolo',
        prefix=[LaunchConfiguration('yolo_python_executable')],
        additional_env={
            'PYTHONNOUSERSITE': '1',
            'YOLO_CONFIG_DIR': '/tmp/wvcsc_ultralytics',
        },
        parameters=[
            os.path.join(vision_share, 'config', 'vision_real.yaml'),
            {'use_sim_time': False},
        ],
        output='screen')
    visual_servo = Node(
        package='wvcsc_visual_servo', executable='visual_servo',
        parameters=[
            os.path.join(real_config, 'visual_servo_real.yaml'),
            {
                'use_sim_time': False,
                'aim_fixed_range_m': ParameterValue(
                    LaunchConfiguration('aim_fixed_range_m'), value_type=float),
                'aim_range_tolerance_m': ParameterValue(
                    LaunchConfiguration('aim_range_tolerance_m'),
                    value_type=float),
                'desired_offset_u_px': ParameterValue(
                    LaunchConfiguration('aim_trim_u_px'), value_type=float),
                'desired_offset_v_px': ParameterValue(
                    LaunchConfiguration('aim_trim_v_px'), value_type=float),
            },
        ],
        output='screen')
    motion_control = Node(
        package='wvcsc_arm_task', executable='motion_control',
        parameters=[arm_motion_parameters], output='screen')
    spray_task = Node(
        package='wvcsc_arm_task', executable='spray_task',
        parameters=[
            os.path.join(real_config, 'arm_task_real.yaml'),
            arm_motion_parameters,
            {
                'spray_working_distance_m': ParameterValue(
                    LaunchConfiguration('aim_fixed_range_m'), value_type=float),
                'observation_mode': LaunchConfiguration('observation_mode'),
            },
            robot_description,
        ],
        output='screen')
    spray_actuator = Node(
        package='wvcsc_arm_task', executable='spray_actuator',
        parameters=[
            os.path.join(real_config, 'spray_actuator_real.yaml'),
            {'use_sim_time': False},
        ],
        output='screen')
    relay_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            controller_share, 'launch', 'controller.launch.py')),
        launch_arguments={
            'config_file': LaunchConfiguration('relay_config_file'),
        }.items(),
    )
    field_route_manager = Node(
        package='wvcsc_bringup', executable='field_route_manager.py',
        parameters=[
            {
                'use_sim_time': False,
                'mission_file': LaunchConfiguration('mission_file'),
                'map_file': LaunchConfiguration('map'),
                'auto_start': True,
                'wait_for_initial_pose': True,
                'wait_for_nav_active': True,
                'wide_relay_channel': 1,
                'arm_relay_channel': 2,
            },
        ],
        remappings=[('/odom', '/ekf_odom')],
        output='screen')
    keyboard = Node(
        package='wvcsc_arm_task', executable='motion_control_keyboard',
        condition=IfCondition(LaunchConfiguration('use_keyboard')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python')),
        DeclareLaunchArgument('use_keyboard', default_value='false'),
        DeclareLaunchArgument(
            'mission_file',
            default_value=latest_field_route()),
        DeclareLaunchArgument('map', default_value=latest_map_yaml()),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('control_mode', default_value='pv'),
        DeclareLaunchArgument('default_speed', default_value='0.5'),
        DeclareLaunchArgument('arm_velocity_scaling', default_value='0.20'),
        DeclareLaunchArgument('arm_acceleration_scaling', default_value='0.20'),
        DeclareLaunchArgument('observation_mode', default_value='joint_presets'),
        DeclareLaunchArgument('c10_mount_xyz', default_value='-0.055 0 -0.10'),
        DeclareLaunchArgument(
            'c10_mount_rpy', default_value='0 -1.57079632679 0'),
        DeclareLaunchArgument('nozzle_mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('nozzle_mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('aim_fixed_range_m', default_value='1.0'),
        DeclareLaunchArgument('aim_range_tolerance_m', default_value='0.05'),
        DeclareLaunchArgument('aim_trim_u_px', default_value='0.0'),
        DeclareLaunchArgument('aim_trim_v_px', default_value='0.0'),
        DeclareLaunchArgument(
            'relay_config_file',
            default_value=os.path.join(
                controller_share, 'config', 'fault.ini')),
        relay_controller,
        yolo,
        visual_servo,
        motion_control,
        spray_task,
        spray_actuator,
        field_route_manager,
        keyboard,
    ])

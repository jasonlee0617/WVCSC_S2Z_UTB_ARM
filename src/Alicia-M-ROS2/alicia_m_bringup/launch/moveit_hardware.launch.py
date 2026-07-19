"""
moveit_hardware.launch.py - MoveIt2 + 真实硬件

用法:
  ros2 launch alicia_m_bringup moveit_hardware.launch.py
  ros2 launch alicia_m_bringup moveit_hardware.launch.py serial_port:=/dev/ttyACM1
  ros2 launch alicia_m_bringup moveit_hardware.launch.py control_mode:=mit mit_kp:=30.0 mit_kd:=1.5
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    full_path = os.path.join(
        get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def _print_launch_config(context):
    port = LaunchConfiguration('serial_port').perform(context)
    mode = LaunchConfiguration('control_mode').perform(context)
    baud = LaunchConfiguration('baudrate').perform(context)
    speed = LaunchConfiguration('default_speed').perform(context)
    pipeline = LaunchConfiguration('planning_pipeline_id').perform(context)
    planner = LaunchConfiguration('planner_id').perform(context)

    print(f'\033[1;34m[INFO] Serial port: {port}\033[0m')
    print(f'\033[1;34m[INFO] Control mode: {mode}\033[0m')
    print(f'\033[1;34m[INFO] Baudrate: {baud}\033[0m')
    print(f'\033[1;34m[INFO] Default speed: {speed} rad/s\033[0m')
    print(f'\033[1;34m[INFO] MoveIt planner: {pipeline}/{planner}\033[0m')
    if mode == 'mit':
        kp = LaunchConfiguration('mit_kp').perform(context)
        kd = LaunchConfiguration('mit_kd').perform(context)
        print(f'\033[1;34m[INFO] MIT Kp={kp}, Kd={kd}\033[0m')
    return []


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            choices=['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyACM3'],
            description='Serial port'),
        DeclareLaunchArgument(
            'control_mode', default_value='pv',
            description='Control mode: pv or mit'),
        DeclareLaunchArgument(
            'baudrate', default_value='1000000',
            description='Serial baudrate'),
        DeclareLaunchArgument(
            'default_speed', default_value='1.0',
            description='Default joint speed (rad/s)'),
        DeclareLaunchArgument(
            'mit_kp', default_value='50.0',
            description='MIT mode Kp (0-500)'),
        DeclareLaunchArgument(
            'mit_kd', default_value='2.0',
            description='MIT mode Kd (0-5)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz'),
        DeclareLaunchArgument(
            'planning_pipeline_id', default_value='ompl',
            description='MoveIt planning pipeline for WVCSC arm tasks'),
        DeclareLaunchArgument(
            'planner_id', default_value='RRTConnectFast',
            description='OMPL planner for WVCSC arm tasks'),
        DeclareLaunchArgument(
            'enable_wvcsc_motion_interface', default_value='false',
            description=(
                'Enable WVCSC motion_control/retime/spray only after Alicia-M '
                'hardware acceleration limits have been confirmed')),
    ]

    serial_port = LaunchConfiguration('serial_port')
    control_mode = LaunchConfiguration('control_mode')
    baudrate = LaunchConfiguration('baudrate')
    default_speed = LaunchConfiguration('default_speed')
    mit_kp = LaunchConfiguration('mit_kp')
    mit_kd = LaunchConfiguration('mit_kd')
    use_rviz = LaunchConfiguration('use_rviz')
    planning_pipeline_id = LaunchConfiguration('planning_pipeline_id')
    planner_id = LaunchConfiguration('planner_id')
    enable_wvcsc_motion_interface = LaunchConfiguration(
        'enable_wvcsc_motion_interface')

    moveit_config_pkg = 'alicia_m_moveit_config'
    moveit_config_share = FindPackageShare(moveit_config_pkg)
    bringup_share = FindPackageShare('alicia_m_bringup')

    # URDF with real hardware plugin
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            moveit_config_share, 'config',
            'alicia_m_v1_1_follower.urdf.xacro'
        ]),
        ' ros2_control_plugin:=alicia_m_driver/AliciaHardwareInterface',
        ' serial_port:=', serial_port,
        ' baudrate:=', baudrate,
        ' control_mode:=', control_mode,
        ' default_speed:=', default_speed,
        ' mit_kp:=', mit_kp,
        ' mit_kd:=', mit_kd,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            moveit_config_share, 'config',
            'alicia_m_v1_1_follower.srdf'
        ]),
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_content, value_type=str)
    }

    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml,
    }
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    pilz_cartesian_limits_yaml = load_yaml(
        moveit_config_pkg, 'config/pilz_cartesian_limits.yaml')
    ompl_planning_yaml = load_yaml(
        moveit_config_pkg, 'config/ompl_planning.yaml')

    planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'ompl': ompl_planning_yaml,
        'pilz_industrial_motion_planner': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            'request_adapters': '',
        },
    }

    moveit_controllers_yaml = load_yaml(
        moveit_config_pkg, 'config/moveit_controllers.yaml')

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    controllers_yaml = PathJoinSubstitution([
        bringup_share, 'config', 'ros2_controllers.yaml'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='log',
        parameters=[robot_description],
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='both',
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'base_link',
            '--ros-args', '--log-level', 'warn',
        ],
        output='log',
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers_yaml,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            pilz_cartesian_limits_yaml,
        ],
    )

    rviz_config_file = PathJoinSubstitution([
        moveit_config_share, 'config', 'moveit.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            joint_limits_yaml,
        ],
        output='log',
        condition=IfCondition(use_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    retime_server_node = Node(
        package='trajectory_retime_server',
        executable='retime_server',
        name='trajectory_retime_server',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'service_name': '/retime_trajectory', 'use_sim_time': False},
        ],
        condition=IfCondition(enable_wvcsc_motion_interface),
        output='screen',
    )

    arm_task_parameters = {
        'base_frame': 'base_link',
        'group_name': 'arm',
        'tool_link': 'tool0',
        'planning_pipeline_id': planning_pipeline_id,
        'planner_id': planner_id,
        'velocity_scaling': 0.1,
        'acceleration_scaling': 0.1,
        'retime_timeout': 5.0,
        'execution_timeout': 60.0,
        'gripper_action': '/gripper_controller/gripper_cmd',
        'gripper_open_position': 0.0,
        'gripper_closed_position': -0.05,
        'gripper_max_effort': 5.0,
        'use_sim_time': False,
    }
    motion_control_node = Node(
        package='wvcsc_arm_task',
        executable='motion_control',
        parameters=[arm_task_parameters],
        condition=IfCondition(enable_wvcsc_motion_interface),
        output='screen',
    )
    spray_task_node = Node(
        package='wvcsc_arm_task',
        executable='spray_task',
        parameters=[arm_task_parameters],
        condition=IfCondition(enable_wvcsc_motion_interface),
        output='screen',
    )

    delayed_controllers = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )
    delayed_arm_task = TimerAction(period=5.0, actions=[spray_task_node])

    return LaunchDescription(
        declared_args + [
            OpaqueFunction(function=_print_launch_config),
            robot_state_publisher_node,
            control_node,
            static_tf_node,
            move_group_node,
            retime_server_node,
            motion_control_node,
            rviz_node,
            delayed_controllers,
            delayed_arm_task,
        ]
    )

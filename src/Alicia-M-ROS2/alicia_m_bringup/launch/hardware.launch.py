"""
hardware.launch.py - Hardware only (ros2_control + controllers, no MoveIt2)

Usage:
  ros2 launch alicia_m_bringup hardware.launch.py
  ros2 launch alicia_m_bringup hardware.launch.py serial_port:=/dev/ttyACM1
  ros2 launch alicia_m_bringup hardware.launch.py control_mode:=mit mit_kp:=30.0 mit_kd:=1.5
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _print_launch_config(context):
    port = LaunchConfiguration('serial_port').perform(context)
    mode = LaunchConfiguration('control_mode').perform(context)
    baud = LaunchConfiguration('baudrate').perform(context)
    speed = LaunchConfiguration('default_speed').perform(context)

    print(f'\033[1;34m[INFO] Serial port: {port}\033[0m')
    print(f'\033[1;34m[INFO] Control mode: {mode}\033[0m')
    print(f'\033[1;34m[INFO] Baudrate: {baud}\033[0m')
    print(f'\033[1;34m[INFO] Default speed: {speed} rad/s\033[0m')
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
    ]

    serial_port = LaunchConfiguration('serial_port')
    control_mode = LaunchConfiguration('control_mode')
    baudrate = LaunchConfiguration('baudrate')
    default_speed = LaunchConfiguration('default_speed')
    mit_kp = LaunchConfiguration('mit_kp')
    mit_kd = LaunchConfiguration('mit_kd')

    moveit_config_pkg = FindPackageShare('alicia_m_moveit_config')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            moveit_config_pkg, 'config',
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

    bringup_pkg = FindPackageShare('alicia_m_bringup')
    controllers_yaml = PathJoinSubstitution([
        bringup_pkg, 'config', 'ros2_controllers.yaml'
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

    delayed_controllers = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    return LaunchDescription(
        declared_args + [
            OpaqueFunction(function=_print_launch_config),
            robot_state_publisher_node,
            control_node,
            delayed_controllers,
        ]
    )

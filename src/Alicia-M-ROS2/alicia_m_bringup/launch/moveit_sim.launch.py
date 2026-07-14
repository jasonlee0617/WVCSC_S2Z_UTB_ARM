"""
moveit_sim.launch.py - MoveIt2 simulation (no real hardware)

Usage:
  ros2 launch alicia_m_bringup moveit_sim.launch.py
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


def _print_launch_info(context):
    print('\033[1;34m[INFO] Simulation mode: using mock hardware (GenericSystem)\033[0m')
    return []


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz'),
    ]

    use_rviz = LaunchConfiguration('use_rviz')

    moveit_config_pkg = 'alicia_m_moveit_config'
    moveit_config_share = FindPackageShare(moveit_config_pkg)
    bringup_share = FindPackageShare('alicia_m_bringup')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            moveit_config_share, 'config',
            'alicia_m_v1_1_follower.urdf.xacro'
        ]),
        ' ros2_control_plugin:=mock_components/GenericSystem',
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
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    pilz_cartesian_limits_yaml = load_yaml(
        moveit_config_pkg, 'config/pilz_cartesian_limits.yaml')

    planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        },
        'pilz_industrial_motion_planner': {
            'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            'request_adapters': '',
        },
    }

    moveit_controllers_yaml = load_yaml(
        moveit_config_pkg, 'config/moveit_controllers.yaml')

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
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
            kinematics_yaml,
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
            kinematics_yaml,
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
            OpaqueFunction(function=_print_launch_info),
            robot_state_publisher_node,
            control_node,
            static_tf_node,
            move_group_node,
            rviz_node,
            delayed_controllers,
        ]
    )

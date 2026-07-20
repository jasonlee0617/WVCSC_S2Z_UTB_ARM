"""Alicia-M ros2_control, MoveIt, OMPL, retiming and Servo for hardware."""

import os
from functools import partial

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package_name, relative_path):
    path = os.path.join(
        get_package_share_directory(package_name), relative_path)
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _after_spawner(event, _context, *, name, success_actions):
    if event.returncode == 0:
        return list(success_actions)
    return [Shutdown(reason=f'{name} exited with code {event.returncode}')]


def _semantic_description(moveit_share):
    path = os.path.join(
        moveit_share, 'config', 'alicia_m_v1_1_follower.srdf')
    with open(path, encoding='utf-8') as stream:
        semantic = stream.read()
    semantic = semantic.replace(
        'name="alicia_m_v1_1_follower"', 'name="wvcsc_utb_alicia"')
    semantic = semantic.replace(
        'base_link="base_link"', 'base_link="alicia_base_link"')
    semantic = semantic.replace(
        'link1="base_link"', 'link1="alicia_base_link"')
    semantic = semantic.replace(
        '<virtual_joint name="virtual_joint" type="fixed" '
        'parent_frame="world" child_link="base_link"/>', '')
    disabled = '''
    <disable_collisions link1="tool0" link2="camera_link" reason="Adjacent"/>
    <disable_collisions link1="link6" link2="camera_link" reason="Mount"/>
    <disable_collisions link1="link7" link2="camera_link" reason="Mount"/>
    <disable_collisions link1="link8" link2="camera_link" reason="Mount"/>
'''
    return semantic.replace('</robot>', f'{disabled}</robot>')


def generate_launch_description():
    description_share = get_package_share_directory('wvcsc_description')
    moveit_share = get_package_share_directory('alicia_m_moveit_config')
    alicia_bringup_share = get_package_share_directory('alicia_m_bringup')

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
            ' c10_mount_xyz:=', LaunchConfiguration('c10_mount_xyz'),
            ' c10_mount_rpy:=', LaunchConfiguration('c10_mount_rpy'),
            ' nozzle_mount_xyz:=', LaunchConfiguration('nozzle_mount_xyz'),
            ' nozzle_mount_rpy:=', LaunchConfiguration('nozzle_mount_rpy'),
        ]), value_type=str),
    }
    robot_description_semantic = {
        'robot_description_semantic': _semantic_description(moveit_share)}

    kinematics = _load_yaml(
        'alicia_m_moveit_config', 'config/kinematics.yaml')
    kinematics['arm']['kinematics_solver_timeout'] = 0.05
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics}
    joint_limits = _load_yaml(
        'alicia_m_moveit_config', 'config/joint_limits.yaml')
    robot_description_planning = {
        'robot_description_planning': joint_limits}
    ompl = _load_yaml(
        'alicia_m_moveit_config', 'config/ompl_planning.yaml')
    planning_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': ompl,
    }
    controllers = _load_yaml(
        'alicia_m_moveit_config', 'config/moveit_controllers.yaml')

    common_moveit = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        planning_pipeline,
        controllers,
        {
            'use_sim_time': False,
            'moveit_manage_controllers': True,
            'trajectory_execution.allowed_execution_duration_scaling': 2.0,
            'trajectory_execution.allowed_goal_duration_margin': 2.0,
            'trajectory_execution.allowed_start_tolerance': 0.01,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        },
    ]

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(
                alicia_bringup_share, 'config', 'ros2_controllers.yaml'),
        ],
        output='screen',
    )
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        parameters=common_moveit, output='screen')
    # system_real 已由 real_sensors 发布统一机器人 TF，因此默认关闭这里的
    # robot_state_publisher。手眼标定等独立机械臂会话显式打开该参数，避免
    # 只有 joint_states 而没有 tool0/camera TF 的隐蔽故障。
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('publish_robot_state')),
        output='screen',
    )
    retime_server = Node(
        package='trajectory_retime_server', executable='retime_server',
        name='trajectory_retime_server',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'service_name': '/retime_trajectory', 'use_sim_time': False},
        ],
        output='screen',
    )

    servo_parameters = _load_yaml(
        'wvcsc_bringup', 'config/real/moveit_servo_real.yaml')
    servo = ComposableNodeContainer(
        name='moveit_servo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[ComposableNode(
            package='moveit_servo',
            plugin='moveit_servo::ServoNode',
            name='servo_node',
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_planning,
                {'moveit_servo': servo_parameters},
                {'butterworth_filter_coeff': 1.05, 'use_sim_time': False},
            ],
        )],
        output='screen',
    )

    spawner_args = [
        '--controller-manager', '/controller_manager',
        '--controller-manager-timeout', '30.0',
        '--switch-timeout', '30.0',
        '--service-call-timeout', '30.0',
    ]
    joint_state = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', *spawner_args],
        output='screen')
    arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', *spawner_args], output='screen')
    gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', *spawner_args], output='screen')

    start_arm = RegisterEventHandler(OnProcessExit(
        target_action=joint_state,
        on_exit=partial(
            _after_spawner, name='joint_state_broadcaster spawner',
            success_actions=[arm]),
    ))
    start_gripper = RegisterEventHandler(OnProcessExit(
        target_action=arm,
        on_exit=partial(
            _after_spawner, name='arm_controller spawner',
            success_actions=[gripper]),
    ))
    stop_on_gripper_failure = RegisterEventHandler(OnProcessExit(
        target_action=gripper,
        on_exit=partial(
            _after_spawner, name='gripper_controller spawner',
            success_actions=[]),
    ))

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(moveit_share, 'config', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            robot_description_planning,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('control_mode', default_value='pv'),
        DeclareLaunchArgument('default_speed', default_value='0.5'),
        DeclareLaunchArgument('c10_mount_xyz', default_value='-0.055 0 -0.10'),
        DeclareLaunchArgument(
            'c10_mount_rpy', default_value='0 -1.57079632679 0'),
        DeclareLaunchArgument('nozzle_mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('nozzle_mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('publish_robot_state', default_value='false'),
        control_node,
        robot_state_publisher,
        joint_state,
        start_arm,
        start_gripper,
        stop_on_gripper_failure,
        move_group,
        retime_server,
        servo,
        rviz,
    ])

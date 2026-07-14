import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package, relative_path):
    path = os.path.join(get_package_share_directory(package), relative_path)
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_arm_control = LaunchConfiguration('enable_arm_control')
    enable_ackermann = LaunchConfiguration('enable_ackermann')
    description_share = get_package_share_directory('wvcsc_description')
    simulation_share = get_package_share_directory('wvcsc_simulation')
    moveit_share = get_package_share_directory('alicia_m_moveit_config')
    gazebo_share = get_package_share_directory('gazebo_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    navigation_share = get_package_share_directory('my_navigation2')
    alicia_model_root = os.path.dirname(get_package_share_directory('alicia_m_descriptions'))
    gazebo_model_path = os.pathsep.join(filter(None, [
        alicia_model_root,
        os.environ.get('GAZEBO_MODEL_PATH'),
    ]))

    xacro_file = os.path.join(description_share, 'urdf', 'wvcsc_utb_alicia.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ', xacro_file,
                     ' alicia_base_link:=alicia_base_link',
                     ' use_collision_meshes:=true',
                     ' enable_arm_control:=', enable_arm_control,
                     ' enable_ackermann:=', enable_ackermann,
                     ' enable_gazebo_ros2_control:=', enable_arm_control,
                     ' gazebo_controllers_file:=',
                     os.path.join(description_share, 'config', 'ros2_controllers.yaml'),
                     ' ros2_control_plugin:=gazebo_ros2_control/GazeboSystem']),
            value_type=str,
        )
    }

    srdf_path = os.path.join(moveit_share, 'config', 'alicia_m_v1_1_follower.srdf')
    with open(srdf_path, encoding='utf-8') as stream:
        semantic = stream.read()
    semantic = semantic.replace('name="alicia_m_v1_1_follower"', 'name="wvcsc_utb_alicia"')
    semantic = semantic.replace('base_link="base_link"', 'base_link="alicia_base_link"')
    semantic = semantic.replace('link1="base_link"', 'link1="alicia_base_link"')
    semantic = semantic.replace(
        '<virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_link"/>', '')
    robot_description_semantic = {'robot_description_semantic': semantic}

    kinematics = load_yaml('alicia_m_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics,
    }
    joint_limits = load_yaml('alicia_m_moveit_config', 'config/joint_limits.yaml')
    moveit_controllers = load_yaml('alicia_m_moveit_config', 'config/moveit_controllers.yaml')
    planning_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        },
    }
    common_moveit = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        joint_limits,
        planning_pipeline,
        moveit_controllers,
        {
            'use_sim_time': True,
            'moveit_manage_controllers': True,
            'trajectory_execution.allowed_execution_duration_scaling': 1.5,
            'trajectory_execution.allowed_goal_duration_margin': 1.0,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        },
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(simulation_share, 'worlds', 'orchard.world'),
            'verbose': 'false',
            'pause': 'true',
        }.items(),
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}], output='screen',
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'wvcsc_utb_alicia',
                   '-x', '0', '-y', '0', '-z', '0'],
        output='screen',
    )
    unpause = ExecuteProcess(
        cmd=['gz', 'topic', '-p', '/gazebo/orchard/world_control', '-m', 'pause: false'],
        output='log',
    )
    vehicle_sim = Node(
        package='wvcsc_simulation', executable='ackermann_sim.py',
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(enable_ackermann),
    )
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        parameters=common_moveit, output='screen',
    )
    retime_server = Node(
        package='trajectory_retime_server', executable='retime_server',
        name='trajectory_retime_server',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'service_name': '/retime_trajectory', 'use_sim_time': True},
        ],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    arm_task_parameters = {
        'base_frame': 'alicia_base_link',
        'group_name': 'arm',
        'tool_link': 'tool0',
        'velocity_scaling': 0.1,
        'acceleration_scaling': 0.1,
        'retime_timeout': 5.0,
        'execution_timeout': 60.0,
        'gripper_action': '/gripper_controller/gripper_cmd',
        'gripper_open_position': 0.0,
        'gripper_closed_position': -0.05,
        'gripper_max_effort': 5.0,
        'use_sim_time': True,
    }
    motion_control = Node(
        package='wvcsc_arm_task', executable='motion_control',
        parameters=[arm_task_parameters],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    joint_state_controller = TimerAction(period=3.0, actions=[
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
             condition=IfCondition(enable_arm_control), output='screen'),
    ])
    arm_controller = TimerAction(period=4.0, actions=[
        Node(package='controller_manager', executable='spawner',
             arguments=['arm_controller', '--controller-manager', '/controller_manager'],
             condition=IfCondition(enable_arm_control), output='screen'),
    ])
    gripper_controller = TimerAction(period=5.0, actions=[
        Node(package='controller_manager', executable='spawner',
             arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
             condition=IfCondition(enable_arm_control), output='screen'),
    ])
    spray_task = TimerAction(period=7.0, actions=[
        Node(package='wvcsc_arm_task', executable='spray_task',
             parameters=[arm_task_parameters],
             condition=IfCondition(enable_arm_control), output='screen'),
    ])
    odom_world = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'odom'],
        output='log',
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': os.path.join(simulation_share, 'maps', 'orchard.yaml'),
            'use_sim_time': 'True',
            'params_file': os.path.join(navigation_share, 'param', 'wtb_nav2_params.yaml'),
            'autostart': 'True',
        }.items(),
        condition=IfCondition(use_nav2),
    )
    initial_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}',
        ],
        condition=IfCondition(use_nav2), output='log',
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(moveit_share, 'config', 'moveit.rviz')],
        parameters=common_moveit, condition=IfCondition(use_rviz), output='log',
    )
    post_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[
            TimerAction(period=5.0, actions=[unpause, vehicle_sim]),
            TimerAction(period=6.0, actions=[nav2, initial_pose]),
        ],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('enable_arm_control', default_value='true'),
        DeclareLaunchArgument('enable_ackermann', default_value='true'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        gazebo,
        state_publisher,
        odom_world,
        spawn,
        post_spawn,
        move_group,
        retime_server,
        motion_control,
        joint_state_controller,
        arm_controller,
        gripper_controller,
        spray_task,
        rviz,
    ])

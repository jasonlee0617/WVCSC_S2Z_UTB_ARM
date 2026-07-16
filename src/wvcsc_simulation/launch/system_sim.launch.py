import os
import socket
from urllib.parse import urlparse

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from wvcsc_simulation.orchard_assets import generate_orchard_assets


def load_yaml(package, relative_path):
    path = os.path.join(get_package_share_directory(package), relative_path)
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def ensure_fresh_gazebo_master(_context):
    """Refuse to attach a new simulation launch to a stale local Gazebo."""
    master_uri = os.environ.get('GAZEBO_MASTER_URI', 'http://127.0.0.1:11345')
    parsed = urlparse(master_uri)
    host = parsed.hostname or '127.0.0.1'
    port = parsed.port or 11345

    if host not in {'127.0.0.1', '::1', 'localhost'}:
        return []

    for family, socktype, protocol, _, address in socket.getaddrinfo(
            host, port, type=socket.SOCK_STREAM):
        with socket.socket(family, socktype, protocol) as sock:
            sock.settimeout(0.2)
            if sock.connect_ex(address) == 0:
                raise RuntimeError(
                    f'Gazebo master {master_uri} is already in use. '
                    'Close the previous Gazebo launch before starting '
                    'wvcsc_simulation; this launch will not attach to an '
                    'existing world.')
    return []


def prepare_orchard(context, simulation_share, base_model_path):
    seed = LaunchConfiguration('orchard_seed').perform(context)
    ratio = LaunchConfiguration('diseased_fruit_ratio').perform(context)
    world = generate_orchard_assets(
        os.path.join(simulation_share, 'worlds', 'orchard.world'),
        os.path.join(simulation_share, 'models', 'apple_tree'),
        seed=seed,
        diseased_ratio=ratio,
    )
    model_path = os.pathsep.join(filter(None, [
        str(world.parent / 'models'),
        base_model_path,
    ]))
    return [
        SetLaunchConfiguration('orchard_world', str(world)),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
    ]


def generate_launch_description():
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    orchard_world = LaunchConfiguration('orchard_world')
    use_nav2_qt = LaunchConfiguration('use_nav2_qt')
    enable_arm_control = LaunchConfiguration('enable_arm_control')
    enable_ackermann = LaunchConfiguration('enable_ackermann')
    use_mock_uav = LaunchConfiguration('use_mock_uav')
    use_replay_uav = LaunchConfiguration('use_replay_uav')
    use_mission_manager = LaunchConfiguration('use_mission_manager')
    use_web_ui = LaunchConfiguration('use_web_ui')
    perception_mode = LaunchConfiguration('perception_mode')
    auto_start_mission = LaunchConfiguration('auto_start_mission')
    return_home_after_finish = LaunchConfiguration('return_home_after_finish')
    mock_target_config = LaunchConfiguration('mock_target_config')
    replay_target_config = LaunchConfiguration('replay_target_config')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    description_share = get_package_share_directory('wvcsc_description')
    simulation_share = get_package_share_directory('wvcsc_simulation')
    arm_task_share = get_package_share_directory('wvcsc_arm_task')
    mission_share = get_package_share_directory('wvcsc_mission_manager')
    uav_share = get_package_share_directory('wvcsc_uav_gateway')
    web_share = get_package_share_directory('wvcsc_web_ui')
    vision_share = get_package_share_directory('wvcsc_rgb_vision')
    visual_servo_share = get_package_share_directory('wvcsc_visual_servo')
    spray_share = get_package_share_directory('wvcsc_spray_controller')
    moveit_share = get_package_share_directory('alicia_m_moveit_config')
    gazebo_share = get_package_share_directory('gazebo_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    navigation_share = get_package_share_directory('my_navigation2')
    alicia_model_root = os.path.dirname(
        get_package_share_directory('alicia_m_descriptions'))
    gazebo_model_path = os.pathsep.join(filter(None, [
        os.path.join(simulation_share, 'models'),
        os.path.dirname(description_share),
        alicia_model_root,
        os.environ.get('GAZEBO_MODEL_PATH'),
    ]))
    gazebo_resource_path = os.pathsep.join(filter(None, [
        os.environ.get('GAZEBO_RESOURCE_PATH'),
        '/usr/share/gazebo-11',
        '/opt/ros/humble/share',
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
                     ' enable_c10_camera:=true',
                     ' enable_c10_gazebo:=true',
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
        '<virtual_joint name="virtual_joint" type="fixed" '
        'parent_frame="world" child_link="base_link"/>', '')
    c10_disabled_collisions = '''
    <disable_collisions link1="tool0" link2="camera_link" reason="Adjacent"/>
    <disable_collisions link1="link6" link2="camera_link" reason="Mount"/>
    <disable_collisions link1="link7" link2="camera_link" reason="Mount"/>
    <disable_collisions link1="link8" link2="camera_link" reason="Mount"/>
'''
    semantic = semantic.replace(
        '</robot>', f'{c10_disabled_collisions}</robot>')
    robot_description_semantic = {'robot_description_semantic': semantic}

    kinematics = load_yaml('alicia_m_moveit_config', 'config/kinematics.yaml')
    kinematics['arm']['kinematics_solver_timeout'] = 0.05
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
    servo_parameters = load_yaml(
        'wvcsc_visual_servo', 'config/moveit_servo.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': orchard_world,
            'verbose': 'false',
            'pause': 'true',
            'gui': gazebo_gui,
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
        parameters=[{
            'use_sim_time': True,
            'wheel_base': 0.67,
            'max_steering_angle': 0.48,
            'max_linear_speed': 0.8,
            'command_timeout': 0.5,
        }], output='screen',
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
        'planning_time': 2.0,
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
    spawner_arguments = [
        '--controller-manager', '/controller_manager',
        '--controller-manager-timeout', '30.0',
        '--switch-timeout', '30.0',
    ]
    joint_state_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', *spawner_arguments],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', *spawner_arguments],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    gripper_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', *spawner_arguments],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    spray_task = Node(
        package='wvcsc_arm_task', executable='spray_task',
        parameters=[
            os.path.join(arm_task_share, 'config', 'arm_task.yaml'),
            arm_task_parameters,
        ],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    moveit_servo = Node(
        package='moveit_servo', executable='servo_node_main',
        name='servo_node',
        parameters=[*common_moveit, {'moveit_servo': servo_parameters}],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    visual_servo = Node(
        package='wvcsc_visual_servo', executable='visual_servo',
        parameters=[
            os.path.join(visual_servo_share, 'config', 'visual_servo.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    start_arm_controller = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_controller,
        on_exit=[arm_controller],
    ))
    start_gripper_controller = RegisterEventHandler(OnProcessExit(
        target_action=arm_controller,
        on_exit=[gripper_controller],
    ))
    start_spray_task = RegisterEventHandler(OnProcessExit(
        target_action=gripper_controller,
        on_exit=[spray_task],
    ))
    world_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world', '--child-frame-id', 'map'],
        output='log',
    )
    map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='log',
    )
    nav2_params = os.path.join(simulation_share, 'config', 'nav2_sim.yaml')
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[nav2_params, {
            'yaml_filename': os.path.join(simulation_share, 'maps', 'orchard.yaml'),
            'use_sim_time': True,
        }],
        condition=IfCondition(use_nav2), output='screen',
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        condition=IfCondition(use_nav2), output='screen',
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(use_nav2),
    )
    mission_manager = Node(
        package='wvcsc_mission_manager', executable='mission_manager',
        parameters=[
            os.path.join(mission_share, 'config', 'mission_manager.yaml'),
            {
                'auto_start': ParameterValue(auto_start_mission, value_type=bool),
                'return_home_after_finish': ParameterValue(
                    return_home_after_finish, value_type=bool),
                'use_sim_time': True,
            },
        ],
        condition=IfCondition(use_mission_manager), output='screen',
    )
    nav2_qt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, 'launch', 'nav2_qt.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'goal_pose_topic': '/manual_goal_pose',
            'road_center_y': '0.0',
            'road_yaw': '0.0',
        }.items(),
        condition=IfCondition(use_nav2_qt),
    )
    mock_uav = Node(
        package='wvcsc_uav_gateway', executable='mock_uav_gateway',
        parameters=[{
            'config_file': mock_target_config,
            'use_sim_time': True,
        }],
        condition=IfCondition(PythonExpression([
            "'", use_mock_uav, "' == 'true' and '",
            use_nav2_qt, "' != 'true'",
        ])), output='screen',
    )
    replay_uav = Node(
        package='wvcsc_uav_gateway', executable='replay_uav_gateway',
        parameters=[{
            'config_file': replay_target_config,
            'use_sim_time': True,
        }],
        condition=IfCondition(PythonExpression([
            "'", use_replay_uav, "' == 'true' and '",
            use_nav2_qt, "' != 'true'",
        ])), output='screen',
    )
    web_ui = Node(
        package='wvcsc_web_ui', executable='web_server',
        parameters=[
            os.path.join(web_share, 'config', 'web_ui.yaml'),
            {
                'host': web_host,
                'port': ParameterValue(web_port, value_type=int),
                'use_sim_time': True,
            },
        ],
        condition=IfCondition(use_web_ui), output='screen',
    )
    spray_simulator = Node(
        package='wvcsc_spray_controller', executable='spray_simulator',
        parameters=[
            os.path.join(spray_share, 'config', 'spray_sim.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    yolo_vision = Node(
        package='wvcsc_rgb_vision', executable='two_stage_yolo',
        parameters=[
            os.path.join(vision_share, 'config', 'vision_sim.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(PythonExpression([
            "'", perception_mode, "' == 'yolo'",
        ])), output='screen',
    )
    mock_vision = Node(
        package='wvcsc_simulation', executable='mock_vision.py',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression([
            "'", perception_mode, "' == 'mock'",
        ])), output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(simulation_share, 'rviz', 'wvcsc.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            joint_limits,
            {'use_sim_time': True},
        ],
        condition=IfCondition(use_rviz), output='log',
    )
    post_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[
            unpause,
            TimerAction(period=0.5, actions=[vehicle_sim]),
            TimerAction(period=0.75, actions=[yolo_vision, mock_vision]),
            TimerAction(period=1.0, actions=[joint_state_controller]),
            TimerAction(period=2.0, actions=[map_server, map_lifecycle]),
            TimerAction(period=3.0, actions=[nav2]),
            TimerAction(
                period=6.0,
                actions=[mission_manager, mock_uav, replay_uav, nav2_qt],
            ),
        ],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('orchard_seed', default_value='42'),
        DeclareLaunchArgument('diseased_fruit_ratio', default_value='0.50'),
        DeclareLaunchArgument('use_nav2_qt', default_value='false'),
        DeclareLaunchArgument('enable_arm_control', default_value='true'),
        DeclareLaunchArgument('enable_ackermann', default_value='true'),
        DeclareLaunchArgument('use_mock_uav', default_value='true'),
        DeclareLaunchArgument('use_replay_uav', default_value='false'),
        DeclareLaunchArgument('use_mission_manager', default_value='true'),
        DeclareLaunchArgument('use_web_ui', default_value='false'),
        DeclareLaunchArgument('perception_mode', default_value='mock'),
        DeclareLaunchArgument('auto_start_mission', default_value='true'),
        DeclareLaunchArgument('return_home_after_finish', default_value='false'),
        DeclareLaunchArgument(
            'mock_target_config',
            default_value=os.path.join(uav_share, 'config', 'mock_targets.yaml')),
        DeclareLaunchArgument(
            'replay_target_config',
            default_value=os.path.join(uav_share, 'config', 'replay_targets.yaml')),
        DeclareLaunchArgument('web_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),
        OpaqueFunction(function=ensure_fresh_gazebo_master),
        OpaqueFunction(
            function=prepare_orchard,
            args=[simulation_share, gazebo_model_path],
        ),
        gazebo,
        state_publisher,
        world_map,
        map_odom,
        spawn,
        post_spawn,
        move_group,
        moveit_servo,
        retime_server,
        motion_control,
        start_arm_controller,
        start_gripper_controller,
        start_spray_task,
        web_ui,
        spray_simulator,
        visual_servo,
        rviz,
    ])

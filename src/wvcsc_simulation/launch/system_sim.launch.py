# system_sim.launch.py
# ============================================================================
# WVCSC 系统仿真顶层启动脚本 (Gazebo Sim Launch)
# ============================================================================
#
# 职责：
# 1. 加载 `wvcsc_description` 描述的复合机器人 URDF 模型。
# 2. 启动 Gazebo，动态生成带有病树的果园仿真世界。
# 3. 通过严密的 `OnProcessExit` 触发和 `TimerAction` 延时，严格保证控制器
#    (joint_state_broadcaster -> arm_controller -> gripper_controller) 顺序生成。
# 4. 实现仿真零重力启动策略：先让控制器加载完毕，再恢复重力，防止机械臂坠落。
# 5. 可选地启动 Nav2、YOLO 感知、视觉伺服、任务管理器等核心模块。
#

import os
import socket
import subprocess
from functools import partial
from urllib.parse import urlparse

import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
from wvcsc_simulation.data_acquisition.orchard_assets import generate_orchard_assets


def load_yaml(package, relative_path):
    """辅助函数：加载 YAML 配置文件。"""
    path = os.path.join(get_package_share_directory(package), relative_path)
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def process_exit_actions(
        event, _context, *, process_name, success_actions):
    """
    启动链式回调：仅当上一个进程成功退出（returncode == 0）时，
    才执行后续成功动作列表。否则直接关闭整个 Launch。
    """
    if event.returncode == 0:
        return list(success_actions)
    return [Shutdown(
        reason=f'{process_name} exited with code {event.returncode}')]


def validate_arm_scaling(context):
    """OpaqueFunction：检查用户传入的机械臂运动速度/加速度缩放是否合法。"""
    for name in ('arm_velocity_scaling', 'arm_acceleration_scaling'):
        raw_value = LaunchConfiguration(name).perform(context)
        try:
            value = float(raw_value)
        except ValueError as error:
            raise RuntimeError(f'{name} must be a number in (0, 1]') from error
        if not 0.0 < value <= 1.0:
            raise RuntimeError(f'{name} must be in (0, 1], got {raw_value}')
    return []


def ensure_fresh_gazebo_master(_context):
    """
    OpaqueFunction：在启动前检查 Gazebo 端口是否已被占用。
    防止将新仿真进程错误地挂载到一个已经被终止的僵尸 Gazebo 实例上。
    """
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


def check_yolo_runtime(context):
    """
    OpaqueFunction：预检查 YOLO 运行时环境是否正常。
    防止因为 PyTorch 版本不兼容导致 Gazebo 启动后延迟报错。
    """
    interpreter = LaunchConfiguration('yolo_python_executable').perform(context)
    if not os.path.isfile(interpreter) or not os.access(interpreter, os.X_OK):
        raise RuntimeError(
            f'YOLO Python interpreter is not executable: {interpreter}. '
            'Set yolo_python_executable to the isolated WVCSC YOLO environment.')

    environment = os.environ.copy()
    environment['PYTHONNOUSERSITE'] = '1'
    environment['YOLO_CONFIG_DIR'] = '/tmp/wvcsc_ultralytics'
    os.makedirs(environment['YOLO_CONFIG_DIR'], exist_ok=True)
    check = subprocess.run(
        [
            interpreter,
            '-c',
            'import cv_bridge, rclpy, torch, torchvision, ultralytics; '
            'print(torch.__version__, torchvision.__version__, ultralytics.__version__)',
        ],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    if check.returncode:
        detail = (check.stderr or check.stdout).strip()
        raise RuntimeError(
            f'YOLO runtime check failed for {interpreter}: {detail}')
    return []


def prepare_orchard(context, simulation_share, base_model_path):
    """
    OpaqueFunction：动态生成果园 SDF 世界文件。
    根据 `orchard_seed` 和 `diseased_fruit_ratio` 参数，
    生成不同分布的病树。
    """
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
    # ------------------------- 1. 声明所有 Launch 参数 -------------------------
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    orchard_world = LaunchConfiguration('orchard_world')
    use_nav2_qt = LaunchConfiguration('use_nav2_qt')
    enable_arm_control = LaunchConfiguration('enable_arm_control')
    enable_ackermann = LaunchConfiguration('enable_ackermann')
    use_mission_manager = LaunchConfiguration('use_mission_manager')
    observation_mode = LaunchConfiguration('observation_mode')
    show_sim_spray_status = LaunchConfiguration('show_sim_spray_status')
    arm_velocity_scaling = LaunchConfiguration('arm_velocity_scaling')
    arm_acceleration_scaling = LaunchConfiguration('arm_acceleration_scaling')
    planning_pipeline_id = LaunchConfiguration('planning_pipeline_id')
    planner_id = LaunchConfiguration('planner_id')
    yolo_python_executable = LaunchConfiguration('yolo_python_executable')
    return_home_after_finish = LaunchConfiguration('return_home_after_finish')

    # 获取各个功能包的共享目录路径
    description_share = get_package_share_directory('wvcsc_description')
    simulation_share = get_package_share_directory('wvcsc_simulation')
    simulation_prefix = get_package_prefix('wvcsc_simulation')
    arm_task_share = get_package_share_directory('wvcsc_arm_task')
    mission_share = get_package_share_directory('wvcsc_mission_manager')
    vision_share = get_package_share_directory('wvcsc_rgb_vision')
    visual_servo_share = get_package_share_directory('wvcsc_visual_servo')
    spray_share = get_package_share_directory('wvcsc_arm_task')
    moveit_share = get_package_share_directory('alicia_m_moveit_config')
    gazebo_share = get_package_share_directory('gazebo_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    bringup_share = get_package_share_directory('wvcsc_bringup')

    # Gazebo 模型路径设置
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
    gazebo_plugin_path = os.pathsep.join(filter(None, [
        os.path.join(simulation_prefix, 'lib'),
        os.environ.get('GAZEBO_PLUGIN_PATH'),
    ]))

    # ------------------------- 2. 机器人模型描述 (URDF & SRDF) -------------------------
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

    # MoveIt 语义 SRDF 文件运行时补丁：
    # 将机械臂的基座从 `base_link` 改写为 `alicia_base_link`，与整车模型匹配。
    srdf_path = os.path.join(moveit_share, 'config', 'alicia_m_v1_1_follower.srdf')
    with open(srdf_path, encoding='utf-8') as stream:
        semantic = stream.read()
    semantic = semantic.replace('name="alicia_m_v1_1_follower"', 'name="wvcsc_utb_alicia"')
    semantic = semantic.replace('base_link="base_link"', 'base_link="alicia_base_link"')
    semantic = semantic.replace('link1="base_link"', 'link1="alicia_base_link"')
    semantic = semantic.replace(
        '<virtual_joint name="virtual_joint" type="fixed" '
        'parent_frame="world" child_link="base_link"/>', '')
    # 插入 C10 相机与机械臂末端的碰撞对排除配置，避免 MoveIt 视为自碰撞
    c10_disabled_collisions = '''
    <disable_collisions link1="tool0" link2="camera_link" reason="Adjacent"/>
    <disable_collisions link1="link6" link2="camera_link" reason="Mount"/>
    <disable_collisions link1="link7" link2="camera_link" reason="Mount"/>
    <disable_collisions link1="link8" link2="camera_link" reason="Mount"/>
    <disable_collisions link1="tool0" link2="spray_nozzle_link" reason="Adjacent"/>
    <disable_collisions link1="base_link" link2="wide_sprayer_link" reason="Adjacent"/>
    <disable_collisions link1="arm_mount_link" link2="wide_sprayer_link" reason="Adjacent"/>
'''
    semantic = semantic.replace(
        '</robot>', f'{c10_disabled_collisions}</robot>')
    robot_description_semantic = {'robot_description_semantic': semantic}

    # 运动学求解器、关节限制、规划器与控制器配置加载
    kinematics = load_yaml('alicia_m_moveit_config', 'config/kinematics.yaml')
    kinematics['arm']['kinematics_solver_timeout'] = 0.05
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics,
    }
    joint_limits = load_yaml('alicia_m_moveit_config', 'config/joint_limits.yaml')
    robot_description_planning = {
        'robot_description_planning': joint_limits,
    }
    moveit_controllers = load_yaml('alicia_m_moveit_config', 'config/moveit_controllers.yaml')
    ompl_planning = load_yaml('alicia_m_moveit_config', 'config/ompl_planning.yaml')
    planning_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': ompl_planning,
    }
    common_moveit = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
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
    # MoveIt Servo 不加载 Kinematics (逆运动学) 求解器，
    # 使得底层采用快速逆雅可比路径，而不是每个控制周期都跑数值 IK。
    servo_moveit = [
        robot_description,
        robot_description_semantic,
        robot_description_planning,
        {'use_sim_time': True},
    ]
    servo_parameters = load_yaml(
        'wvcsc_visual_servo', 'config/moveit_servo.yaml')

    # ------------------------- 3. 基础仿真组件 (Gazebo, Spawn, RSP) -------------------------
    # Gazebo Humble's stock ``gazebo.launch.py`` starts gzclient with the
    # end-of-life banner plugin.  On this Gazebo Classic/Qt/OGRE combination
    # that client can remain as a 1x1 render window or fail before presenting
    # its main window.  Keep the standard ROS-aware server launcher, but start
    # an unmodified Gazebo Classic client for a reliable desktop GUI.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': orchard_world,
            'verbose': 'false',
            'pause': 'true',        # 必须从暂停状态启动，以便在零重力环境下加载控制器
        }.items(),
    )
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gazebo_gui),
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

    # ------------------------- 4. 仿真重力控制策略 (极其关键) -------------------------
    # 为了防止机械臂在控制器未加载前因重力坠落，采用：设定零重力 -> 生成控制器 -> 恢复重力
    zero_gravity = ExecuteProcess(
        cmd=[
            'gz', 'topic', '-p', '/gazebo/orchard/physics',
            '-m', 'gravity { x: 0 y: 0 z: 0 }',
        ],
        output='log',
        condition=IfCondition(enable_arm_control),
    )
    unpause_with_zero_gravity = ExecuteProcess(
        cmd=['gz', 'topic', '-p', '/gazebo/orchard/world_control', '-m', 'pause: false'],
        output='log',
        condition=IfCondition(enable_arm_control),
    )
    restore_gravity = ExecuteProcess(
        cmd=[
            'gz', 'topic', '-p', '/gazebo/orchard/physics',
            '-m', 'gravity { x: 0 y: 0 z: -9.8 }',
        ],
        output='log',
        condition=IfCondition(enable_arm_control),
    )
    unpause_without_arm = ExecuteProcess(
        cmd=['gz', 'topic', '-p', '/gazebo/orchard/world_control', '-m', 'pause: false'],
        output='log',
        condition=UnlessCondition(enable_arm_control),
    )

    # ------------------------- 5. 车辆仿真与机械臂控制节点 -------------------------
    vehicle_sim = Node(
        package='wvcsc_simulation', executable='ackermann_sim.py',
        parameters=[{
            'use_sim_time': True,
            'wheel_base': 0.82,
            'max_steering_angle': 0.48,
            'max_linear_speed': 0.35,
            # velocity_smoother 以 20 Hz 发布 /cmd_vel。超时必须明显大于
            # 50 ms 的传输周期，否则正常的调度抖动也会被误判为失联，导致
            # 车辆交替刹车并触发 Nav2 控制失败。正常停车由 smoother 主动
            # 发布的零速度完成；0.25 s 只作为节点失联时的后备制动。
            'command_timeout': 0.25,
            'cmd_angular_mode': 'yaw_rate',
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
    # 机械臂基础参数传递
    arm_task_parameters = {
        'base_frame': 'alicia_base_link',
        'group_name': 'arm',
        'tool_link': 'tool0',
        'planning_pipeline_id': planning_pipeline_id,
        'planner_id': planner_id,
        'velocity_scaling': ParameterValue(
            arm_velocity_scaling, value_type=float),
        'acceleration_scaling': ParameterValue(
            arm_acceleration_scaling, value_type=float),
        'retime_service_name': '/retime_trajectory',
        'retime_timeout': 5.0,
        'execution_timeout': 60.0,
        'planning_time': 2.0,
        'gripper_action': '/gripper_controller/gripper_cmd',
        'gripper_open_position': 0.0,
        'gripper_closed_position': -0.05,
        'gripper_max_effort': 5.0,
        # 仿真默认走与实机相同的 IK 观察状态机；预设姿态仅作为显式回归入口。
        'observation_mode': observation_mode,
        # 仿真不允许视觉对准失败后盲喷，必须把未完成目标记为 unresolved。
        'spray_on_alignment_failure': False,
        'use_sim_time': True,
    }
    motion_control = Node(
        package='wvcsc_arm_task', executable='motion_control',
        parameters=[arm_task_parameters],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    # 控制器生成参数 (要求每个生成器必须在 30 秒内完成)
    spawner_arguments = [
        '--controller-manager', '/controller_manager',
        '--controller-manager-timeout', '30.0',
        '--switch-timeout', '30.0',
        '--service-call-timeout', '30.0',
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
            robot_description,
        ],
        condition=IfCondition(enable_arm_control), output='screen',
    )

    # ------------------------- 6. 视觉伺服与 MoveIt Servo -------------------------
    # 将 MoveIt Servo 打包成 ComposableNodeContainer (组合节点容器)
    moveit_servo = ComposableNodeContainer(
        name='moveit_servo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoNode',
                name='servo_node',
                parameters=[
                    *servo_moveit,
                    {'moveit_servo': servo_parameters},
                    {'butterworth_filter_coeff': 1.05},
                ],
            ),
        ],
        condition=IfCondition(enable_arm_control),
        output='screen',
    )
    visual_servo = Node(
        package='wvcsc_visual_servo', executable='visual_servo',
        parameters=[
            os.path.join(visual_servo_share, 'config', 'visual_servo.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    # 喷洒仿真模拟器
    spray_actuator = Node(
        package='wvcsc_arm_task', executable='spray_actuator',
        parameters=[
            os.path.join(spray_share, 'config', 'spray_actuator.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(enable_arm_control), output='screen',
    )
    sim_relay = Node(
        package='wvcsc_simulation', executable='sim_relay.py',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression([
            "'", use_mission_manager, "' == 'true' or '",
            enable_arm_control, "' == 'true'",
        ])), output='screen',
    )
    # Relay channel state is a required safety and visualisation dependency for
    # the default mission path.  A script that exits cleanly without spinning
    # is still a failure; terminate the launch rather than silently continuing
    # with every wide-spray request dropped.
    guard_sim_relay = RegisterEventHandler(OnProcessExit(
        target_action=sim_relay,
        on_exit=lambda event, _context: [Shutdown(
            reason=f'simulation relay exited unexpectedly with code '
                   f'{event.returncode}')],
    ))

    # ------------------------- 7. 控制器顺序启动链 (事件驱动) -------------------------
    # 使用 OnProcessExit 构建严格的链式依赖关系：
    # joint_state_broadcaster 成功启动 -> arm_controller 启动
    start_arm_controller = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_controller,
        on_exit=partial(
            process_exit_actions,
            process_name='joint_state_broadcaster spawner',
            success_actions=[arm_controller]),
    ))
    # physics 在零重力下 unpause (取消暂停)
    start_zero_gravity_physics = RegisterEventHandler(OnProcessExit(
        target_action=zero_gravity,
        on_exit=[unpause_with_zero_gravity],
    ))
    # arm_controller 成功启动 -> gripper_controller 启动
    start_gripper_controller = RegisterEventHandler(OnProcessExit(
        target_action=arm_controller,
        on_exit=partial(
            process_exit_actions,
            process_name='arm_controller spawner',
            success_actions=[gripper_controller]),
    ))
    # gripper_controller 成功启动 -> 恢复重力 -> 启动喷洒任务节点
    start_spray_task = RegisterEventHandler(OnProcessExit(
        target_action=gripper_controller,
        on_exit=partial(
            process_exit_actions,
            process_name='gripper_controller spawner',
            success_actions=[restore_gravity, spray_task]),
    ))

    # ------------------------- 8. TF 静态变换 (世界坐标与地图对齐) -------------------------
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

    # ------------------------- 9. Nav2 导航栈 -------------------------
    nav2_params_source = os.path.join(simulation_share, 'config', 'nav2_sim.yaml')
    nav2_params = RewrittenYaml(
        source_file=nav2_params_source,
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(
                simulation_share, 'config', 'behavior_trees',
                'navigate_route.xml'),
        },
        convert_types=True,
    )
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

    # ------------------------- 10. 任务管理与感知 (Qt/RViz, YOLO) -------------------------
    mission_manager = Node(
        package='wvcsc_mission_manager', executable='mission_manager',
        parameters=[
            os.path.join(mission_share, 'config', 'mission_manager.yaml'),
            {
                'return_home_after_finish': ParameterValue(
                    return_home_after_finish, value_type=bool),
                # Alicia 在车顶以 pi yaw 安装；手动树点必须按该真实安装姿态
                # 解释，才能和 Qt/RViz 记录的 map 坐标一致。
                'arm_base_yaw_rad': 3.141592653589793,
                # 仿真 map->odom 为静态单位变换，因此可用新鲜 /odom 对近目标
                # Nav2 abort 进行停靠质量复核；实机仍使用 AMCL 默认配置。
                'require_docking_quality': True,
                'docking_pose_source': 'odom',
                'accept_aborted_near_goal': True,
                'nav_goal_xy_tolerance_m': 0.08,
                'nav_goal_yaw_tolerance_rad': 0.10,
                'max_docking_position_error_m': 0.10,
                'max_docking_yaw_error_rad': 0.12,
                'nav_goal_timeout_sec': 45.0,
                'inspect_nav_behavior_tree': os.path.join(
                    simulation_share, 'config', 'behavior_trees',
                    'navigate_inspect.xml'),
                'route_nav_behavior_tree': os.path.join(
                    simulation_share, 'config', 'behavior_trees',
                    'navigate_route.xml'),
                'use_sim_time': True,
            },
        ],
        condition=IfCondition(use_mission_manager), output='screen',
    )
    nav2_qt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2_qt.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'goal_pose_topic': '/manual_goal_pose',
            # Gazebo 使用静态 map->odom，不运行 AMCL 的全局重定位服务；仍要求
            # 操作员在 RViz 重新给出初始位姿后才能记录任务起点。
            'require_global_relocalization_service': 'false',
            'show_sim_spray_status': show_sim_spray_status,
            # The simulation map contains the same circular trunk envelope as
            # Gazebo.  Reject manually-recorded inspect parking poses that
            # would be inside its static inflation cost before Nav2 starts.
            'simulation_parking_clearance_check': 'true',
            'observation_mode': observation_mode,
        }.items(),
        condition=IfCondition(use_nav2_qt),
    )
    yolo_vision = Node(
        package='wvcsc_rgb_vision', executable='perception_pipeline',
        prefix=[yolo_python_executable],  # 指定独立的 Python 虚拟环境解释器
        additional_env={
            'PYTHONNOUSERSITE': '1',
            'YOLO_CONFIG_DIR': '/tmp/wvcsc_ultralytics',
        },
        parameters=[
            os.path.join(vision_share, 'config', 'vision_sim.yaml'),
            {'use_sim_time': True},
        ],
        on_exit=[Shutdown(reason='YOLO perception node exited')],
        output='screen',
    )

    # ------------------------- 11. 可视化 (RViz2) -------------------------
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(simulation_share, 'rviz', 'wvcsc.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            robot_description_planning,
            {'use_sim_time': True},
        ],
        condition=IfCondition(use_rviz), output='log',
    )

    # ------------------------- 12. 后处理启动链 (OnProcessExit) -------------------------
    post_spawn = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[
            zero_gravity,                               # 生成模型后立刻将重力置零
            unpause_without_arm,                       # 如果没有机械臂，直接取消暂停
            TimerAction(
                period=0.5,
                actions=[vehicle_sim],
            ),
            TimerAction(period=0.75, actions=[yolo_vision]),  # 0.75秒后启动 YOLO
            TimerAction(period=1.5, actions=[joint_state_controller]), # 1.5秒后启动关节广播
            TimerAction(period=2.0, actions=[map_server, map_lifecycle]), # 2.0秒后启动地图服务器
            TimerAction(period=3.0, actions=[nav2]),    # 3.0秒后启动导航栈
            TimerAction(
                period=6.0,
                actions=[mission_manager, nav2_qt],
            ),
        ],
    ))

    # ------------------------- 13. LaunchDescription 组装 -------------------------
    return LaunchDescription([
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('orchard_seed', default_value='42'),
        DeclareLaunchArgument('diseased_fruit_ratio', default_value='0.50'),
        DeclareLaunchArgument('use_nav2_qt', default_value='true'),
        DeclareLaunchArgument('enable_arm_control', default_value='true'),
        DeclareLaunchArgument('enable_ackermann', default_value='true'),
        DeclareLaunchArgument('use_mission_manager', default_value='true'),
        DeclareLaunchArgument('observation_mode', default_value='ik'),
        DeclareLaunchArgument('show_sim_spray_status', default_value='true'),
        DeclareLaunchArgument('arm_velocity_scaling', default_value='0.40'),
        DeclareLaunchArgument('arm_acceleration_scaling', default_value='0.50'),
        DeclareLaunchArgument('planning_pipeline_id', default_value='ompl'),
        DeclareLaunchArgument('planner_id', default_value='RRTConnectFast'),
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python')),
        DeclareLaunchArgument('return_home_after_finish', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        # 前置安全检查 (若检查失败，直接终止 Launch 启动)
        OpaqueFunction(function=validate_arm_scaling),
        OpaqueFunction(function=ensure_fresh_gazebo_master),
        OpaqueFunction(function=check_yolo_runtime),
        # 动态生成果园世界
        OpaqueFunction(
            function=prepare_orchard,
            args=[simulation_share, gazebo_model_path],
        ),
        gazebo_server,
        gazebo_client,
        guard_sim_relay,
        sim_relay,
        state_publisher,
        world_map,
        map_odom,
        spawn,
        post_spawn,
        start_zero_gravity_physics,
        move_group,
        retime_server,
        moveit_servo,
        motion_control,
        start_arm_controller,
        start_gripper_controller,
        start_spray_task,
        spray_actuator,
        visual_servo,
        rviz,
    ])

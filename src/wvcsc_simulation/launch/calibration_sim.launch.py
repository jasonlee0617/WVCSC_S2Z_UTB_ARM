"""Gazebo Classic vehicle-mounted Alicia-M/C10 eye-in-hand calibration."""

import os
import re
import subprocess
from functools import partial
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tf_transformations import (
    concatenate_matrices,
    euler_from_matrix,
    euler_matrix,
    identity_matrix,
    translation_from_matrix,
    translation_matrix,
)


def _load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _sanitize_robot_description(description):
    """Drop XML comments before Gazebo passes the URDF through RCL parsing."""
    sanitized = re.sub(r'<!--.*?-->', '', str(description), flags=re.DOTALL)
    if '<robot' not in sanitized or '</robot>' not in sanitized:
        raise RuntimeError('xacro did not produce a complete robot description')
    return sanitized


def _generate_robot_description(xacro_file, controllers_file):
    result = subprocess.run(
        [
            'xacro', xacro_file,
            f'gazebo_controllers_file:={controllers_file}',
            'enable_arm_control:=true',
            # Calibration keeps the chassis stationary, but MoveIt still needs
            # all six vehicle joints on /joint_states for a complete state.
            'enable_ackermann:=true',
            'enable_gazebo_ros2_control:=true',
            'enable_c10_camera:=true',
            'enable_c10_gazebo:=true',
            'use_collision_meshes:=true',
            'calibration_fix_base:=true',
            'ros2_control_plugin:=gazebo_ros2_control/GazeboSystem',
            'c10_noise_stddev:=0.0',
        ],
        check=False, capture_output=True, text=True)
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()
        raise RuntimeError(f'calibration xacro failed: {detail}')
    return _sanitize_robot_description(result.stdout)


def _semantic_description(moveit_share):
    path = os.path.join(moveit_share, 'config', 'alicia_m_v1_1_follower.srdf')
    with open(path, encoding='utf-8') as stream:
        semantic = stream.read()
    semantic = semantic.replace(
        'name="alicia_m_v1_1_follower"', 'name="wvcsc_utb_alicia"')
    semantic = semantic.replace('base_link="base_link"',
                                'base_link="alicia_base_link"')
    semantic = semantic.replace('link1="base_link"',
                                'link1="alicia_base_link"')
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


def _after_success(event, _context, *, process_name, success_actions):
    if event.returncode == 0:
        return list(success_actions)
    return [Shutdown(reason=f'{process_name} exited with code {event.returncode}')]


def _marker_position(simulation_parameters):
    values = tuple(float(value) for value in simulation_parameters[
        'marker_position_base_m'])
    if len(values) != 3:
        raise RuntimeError('marker_position_base_m must contain three values')
    return values


def _marker_spawn_pose(robot_description, marker_position):
    """Express an Alicia-base marker pose in Gazebo's retained root link."""
    root = ElementTree.fromstring(robot_description)
    by_child = {}
    for joint in root.findall('joint'):
        child = joint.find('child')
        if child is not None:
            by_child[child.attrib['link']] = joint

    chain = []
    link = 'alicia_base_link'
    while link != 'base_footprint':
        joint = by_child.get(link)
        if joint is None or joint.attrib.get('type') != 'fixed':
            raise RuntimeError(
                'robot description has no fixed base_footprint to '
                'alicia_base_link chain')
        chain.append(joint)
        link = joint.find('parent').attrib['link']

    transform = identity_matrix()
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = tuple(float(value) for value in (
            origin.attrib.get('xyz', '0 0 0').split()))
        rpy = tuple(float(value) for value in (
            origin.attrib.get('rpy', '0 0 0').split()))
        transform = concatenate_matrices(
            transform, translation_matrix(xyz), euler_matrix(*rpy))
    transform = concatenate_matrices(
        transform,
        translation_matrix(marker_position),
        euler_matrix(1.57079632679, 0.0, 0.0),
    )
    return translation_from_matrix(transform), euler_from_matrix(transform)


def generate_launch_description():
    simulation_share = get_package_share_directory('wvcsc_simulation')
    calibration_share = get_package_share_directory('wvcsc_calibration')
    description_share = get_package_share_directory('wvcsc_description')
    moveit_share = get_package_share_directory('alicia_m_moveit_config')
    gazebo_share = get_package_share_directory('gazebo_ros')
    alicia_share = get_package_share_directory('alicia_m_descriptions')

    controllers_file = os.path.join(
        description_share, 'config', 'ros2_controllers.yaml')
    xacro_file = os.path.join(
        description_share, 'urdf', 'wvcsc_utb_alicia.urdf.xacro')
    urdf = _generate_robot_description(xacro_file, controllers_file)
    robot_description = {'robot_description': urdf}
    robot_description_semantic = {
        'robot_description_semantic': _semantic_description(moveit_share)}
    kinematics = _load_yaml('alicia_m_moveit_config', 'config/kinematics.yaml')
    kinematics['arm']['kinematics_solver_timeout'] = 0.05
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics}
    robot_description_planning = {
        'robot_description_planning': _load_yaml(
            'alicia_m_moveit_config', 'config/joint_limits.yaml')}
    planning_pipeline = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': _load_yaml('alicia_m_moveit_config', 'config/ompl_planning.yaml'),
    }
    moveit_controllers = _load_yaml(
        'alicia_m_moveit_config', 'config/moveit_controllers.yaml')
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
            'trajectory_execution.allowed_start_tolerance': 0.01,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        },
    ]

    simulation_parameters = _load_yaml(
        'wvcsc_calibration', 'config/auto_handeye_alicia_sim.yaml')[
            'auto_calibration_collector']['ros__parameters']
    marker_position = _marker_position(simulation_parameters)
    marker_xyz, marker_rpy = _marker_spawn_pose(urdf, marker_position)
    marker_model = os.path.join(
        simulation_share, 'models', 'aruco_marker', 'model.sdf')

    gazebo_model_path = os.pathsep.join(filter(None, [
        os.path.join(simulation_share, 'models'),
        os.path.dirname(description_share),
        os.path.dirname(alicia_share),
        '/usr/share/gazebo-11/models',
        os.environ.get('GAZEBO_MODEL_PATH'),
    ]))
    gazebo_resource_path = os.pathsep.join(filter(None, [
        os.environ.get('GAZEBO_RESOURCE_PATH'),
        '/usr/share/gazebo-11',
        '/opt/ros/humble/share',
    ]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(
                simulation_share, 'worlds', 'calibration_vehicle.world'),
            'verbose': 'false',
            'pause': 'true',
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}], output='screen')
    spawn_vehicle = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description', '-entity', 'wvcsc_calibration_vehicle',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen')
    spawn_marker = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-file', marker_model, '-entity', 'calibration_aruco',
            '-reference_frame', 'wvcsc_calibration_vehicle::base_footprint',
            '-x', str(marker_xyz[0]), '-y', str(marker_xyz[1]),
            '-z', str(marker_xyz[2]),
            '-R', str(marker_rpy[0]), '-P', str(marker_rpy[1]),
            '-Y', str(marker_rpy[2]),
        ],
        output='screen')
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        parameters=common_moveit, output='screen')

    spawner_args = [
        '--controller-manager', '/controller_manager',
        '--controller-manager-timeout', '30.0',
        '--switch-timeout', '30.0',
        '--service-call-timeout', '30.0',
    ]
    joint_state = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', *spawner_args], output='screen')
    arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', *spawner_args], output='screen')
    gripper = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', *spawner_args], output='screen')
    unpause = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/unpause_physics',
             'std_srvs/srv/Empty', '{}'],
        output='screen')

    motion_control = Node(
        package='wvcsc_arm_task', executable='motion_control',
        parameters=[{
            'base_frame': 'alicia_base_link',
            'group_name': 'arm',
            'tool_link': 'tool0',
            'planning_pipeline_id': 'ompl',
            'planner_id': 'RRTConnectFast',
            'velocity_scaling': 0.20,
            'acceleration_scaling': 0.20,
            'planning_time': 5.0,
            'execution_timeout': 90.0,
            'use_sim_time': True,
        }],
        output='screen')
    aruco = Node(
        package='ros2_aruco', executable='aruco_node',
        parameters=[{
            'marker_size': 0.070,
            'aruco_dictionary_id': 'DICT_5X5_250',
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'camera_frame': 'camera_color_optical_frame',
            'use_sim_time': True,
        }],
        output='screen')
    marker_tf = Node(
        package='wvcsc_calibration', executable='marker_tf',
        parameters=[{
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'calibration_aruco',
            'marker_id': 1,
            'aruco_topic': '/aruco_markers',
            'smoothing_window': 15,
            'stable_pose_topic': '/calibration/stable_marker_pose',
            'stable_pose_hold_sec': 1.0,
            'use_sim_time': True,
        }],
        output='screen')
    handeye_server = Node(
        package='easy_handeye2', executable='handeye_server',
        name='handeye_server',
        parameters=[{
            'name': 'alicia_c10_eye_in_hand',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'alicia_base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'calibration_aruco',
            'use_sim_time': True,
        }],
        output='screen')
    collector = Node(
        package='wvcsc_calibration', executable='auto_calibration_collector',
        parameters=[
            os.path.join(calibration_share, 'config', 'auto_handeye_alicia.yaml'),
            os.path.join(
                calibration_share, 'config', 'auto_handeye_alicia_sim.yaml'),
        ],
        output='screen')

    # -----------------------------------------------------------------------
    # LEGACY DESK CALIBRATION ENVIRONMENT - REFERENCE ONLY
    #
    # The original desk + standalone-arm setup remains in the repository for
    # historical comparison and manual rollback.  It is intentionally not an
    # active launch branch: it omits the vehicle roof mount and chassis
    # collision geometry, so its safe poses cannot be migrated one-to-one to
    # the vehicle.  To restore it manually, replace the active xacro/world and
    # spawn path below as one set.  Never start both sets of state publishers,
    # controller managers, MoveIt nodes, or Gazebo entities together.
    #
    # legacy_xacro_file = os.path.join(
    #     calibration_share, 'xacro', 'calibration_arm_camera.urdf.xacro')
    # legacy_urdf = _generate_robot_description(legacy_xacro_file, controllers_file)
    # legacy_world = os.path.join(
    #     simulation_share, 'worlds', 'calibration_table.world')
    # legacy_spawn = Node(
    #     package='gazebo_ros', executable='spawn_entity.py',
    #     arguments=['-topic', '/robot_description', '-entity',
    #                'alicia_calibration', '-x', '0', '-y', '0', '-z', '0'],
    #     output='screen')
    # -----------------------------------------------------------------------

    # Gazebo starts paused to make both vehicle and marker spawn deterministic.
    # The calibration world has zero gravity, so unpausing before controllers
    # activate is safe and gives ros2_control an update cycle.
    start_marker = RegisterEventHandler(OnProcessExit(
        target_action=spawn_vehicle,
        on_exit=partial(
            _after_success, process_name='vehicle entity spawn',
            success_actions=[spawn_marker])))
    start_unpause = RegisterEventHandler(OnProcessExit(
        target_action=spawn_marker,
        on_exit=partial(
            _after_success, process_name='ArUco entity spawn',
            success_actions=[unpause])))
    start_joint_state = RegisterEventHandler(OnProcessExit(
        target_action=unpause,
        on_exit=partial(
            _after_success, process_name='Gazebo unpause',
            success_actions=[joint_state])))
    start_arm = RegisterEventHandler(OnProcessExit(
        target_action=joint_state,
        on_exit=partial(
            _after_success, process_name='joint_state_broadcaster spawner',
            success_actions=[arm])))
    start_gripper = RegisterEventHandler(OnProcessExit(
        target_action=arm,
        on_exit=partial(
            _after_success, process_name='arm_controller spawner',
            success_actions=[gripper])))
    start_calibration = RegisterEventHandler(OnProcessExit(
        target_action=gripper,
        on_exit=partial(
            _after_success, process_name='gripper_controller spawner',
            success_actions=[
                motion_control, aruco, marker_tf, handeye_server, collector,
            ])))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),
        gazebo,
        state_publisher,
        spawn_vehicle,
        move_group,
        start_marker,
        start_unpause,
        start_joint_state,
        start_arm,
        start_gripper,
        start_calibration,
    ])

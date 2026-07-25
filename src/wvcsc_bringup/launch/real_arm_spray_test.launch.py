"""Real Alicia-M spray-flow test without vehicle navigation."""

import os
from functools import partial
import importlib.util

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def _include(launch_dir, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, filename)),
        launch_arguments=(arguments or {}).items(),
    )


def _real_mission_helpers(launch_dir):
    path = os.path.join(launch_dir, 'real_system_mission.launch.py')
    spec = importlib.util.spec_from_file_location(
        'wvcsc_real_system_mission_helpers', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch(context, *, launch_dir):
    bringup_share = get_package_share_directory('wvcsc_bringup')
    description_share = get_package_share_directory('wvcsc_description')
    vision_share = get_package_share_directory('wvcsc_rgb_vision')
    controller_share = get_package_share_directory('controller_pkg')
    real_config = os.path.join(bringup_share, 'config', 'real')
    helpers = _real_mission_helpers(launch_dir)
    handeye_path = helpers._resolve_handeye_calibration(
        LaunchConfiguration('handeye_calibration').perform(context))
    c10_xyz, c10_rpy = helpers._load_calibrated_mount(handeye_path)
    # This standalone test intentionally treats tool0 as the spray centerline.
    # Keep the URDF nozzle link at the identity transform for shared launch
    # compatibility, but aim and plan from tool0 itself.
    nozzle_xyz = (0.0, 0.0, 0.0)
    nozzle_rpy = (0.0, 0.0, 0.0)
    c10_mount_xyz = ' '.join(f'{value:.12g}' for value in c10_xyz)
    c10_mount_rpy = ' '.join(f'{value:.12g}' for value in c10_rpy)
    nozzle_mount_xyz = ' '.join(f'{value:.12g}' for value in nozzle_xyz)
    nozzle_mount_rpy = ' '.join(f'{value:.12g}' for value in nozzle_rpy)

    shared_description_args = {
        'serial_port': LaunchConfiguration('serial_port'),
        'baudrate': LaunchConfiguration('baudrate'),
        'control_mode': LaunchConfiguration('control_mode'),
        'default_speed': LaunchConfiguration('default_speed'),
        'c10_mount_xyz': c10_mount_xyz,
        'c10_mount_rpy': c10_mount_rpy,
        'nozzle_mount_xyz': nozzle_mount_xyz,
        'nozzle_mount_rpy': nozzle_mount_rpy,
    }
    xacro_file = os.path.join(
        description_share, 'urdf', 'wvcsc_utb_alicia.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' alicia_base_link:=alicia_base_link',
            ' use_collision_meshes:=true',
            ' enable_arm_control:=true',
            # This launch owns only the arm; the chassis controller is not
            # started, so do not add unreported wheel joints to MoveIt.
            ' enable_ackermann:=false',
            ' enable_gazebo_ros2_control:=false',
            ' enable_c10_camera:=true',
            ' enable_c10_gazebo:=false',
            ' ros2_control_plugin:=alicia_m_driver/AliciaHardwareInterface',
            ' serial_port:=', LaunchConfiguration('serial_port'),
            ' baudrate:=', LaunchConfiguration('baudrate'),
            ' control_mode:=', LaunchConfiguration('control_mode'),
            ' default_speed:=', LaunchConfiguration('default_speed'),
            # Keep vector-valued xacro arguments as one quoted shell token.
            # Without the quotes a negative first component (for example the
            # calibrated RPY value -2.11) is parsed by xacro as an option.
            ' c10_mount_xyz:="', c10_mount_xyz, '"',
            ' c10_mount_rpy:="', c10_mount_rpy, '"',
            ' nozzle_mount_xyz:="', nozzle_mount_xyz, '"',
            ' nozzle_mount_rpy:="', nozzle_mount_rpy, '"',
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

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                controller_share, 'launch', 'controller.launch.py')),
            launch_arguments={
                'config_file': LaunchConfiguration('relay_config_file'),
            }.items(),
        ),
        _include(launch_dir, 'real_arm.launch.py', {
            **shared_description_args,
            'publish_robot_state': 'true',
            'use_rviz': LaunchConfiguration('use_moveit_rviz'),
        }),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('wvcsc_c10_camera'),
                'launch', 'c10_camera.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('c10_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }.items(),
        ),
        Node(
            package='wvcsc_rgb_vision', executable='perception_pipeline',
            prefix=[LaunchConfiguration('yolo_python_executable')],
            additional_env={
                'PYTHONNOUSERSITE': '1',
                'YOLO_CONFIG_DIR': '/tmp/wvcsc_ultralytics',
            },
            parameters=[
                os.path.join(vision_share, 'config', 'vision_real.yaml'),
                {'use_sim_time': False},
            ],
            output='screen'),
        Node(
            package='wvcsc_visual_servo', executable='visual_servo',
            parameters=[
                os.path.join(real_config, 'visual_servo_real.yaml'),
                {
                    'use_sim_time': False,
                    'aim_nozzle_frame': 'tool0',
                },
            ],
            output='screen'),
        Node(
            package='wvcsc_arm_task', executable='motion_control',
            parameters=[arm_motion_parameters], output='screen'),
        Node(
            package='wvcsc_arm_task', executable='spray_task',
            parameters=[
                os.path.join(real_config, 'arm_task_real.yaml'),
                arm_motion_parameters,
                {'observation_mode': LaunchConfiguration('observation_mode')},
                robot_description,
            ],
            output='screen'),
        Node(
            package='wvcsc_arm_task', executable='spray_actuator',
            parameters=[
                os.path.join(real_config, 'spray_actuator_real.yaml'),
                {'use_sim_time': False},
            ],
            output='screen'),
        Node(
            package='wvcsc_bringup', executable='arm_spray_test_qt.py',
            condition=IfCondition(LaunchConfiguration('use_qt_gui')),
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'alicia_base_link',
            }],
            output='screen'),
    ]


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('wvcsc_bringup'), 'launch')
    controller_share = get_package_share_directory('controller_pkg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'c10_device',
            default_value='/dev/video0'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='package://wvcsc_c10_camera/config/c10_intrinsics.yaml'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('control_mode', default_value='pv'),
        DeclareLaunchArgument('default_speed', default_value='0.5'),
        DeclareLaunchArgument(
            'handeye_calibration',
            default_value='latest_real'),
        DeclareLaunchArgument(
            'relay_config_file',
            default_value=os.path.join(
                controller_share, 'config', 'fault.ini')),
        DeclareLaunchArgument('arm_velocity_scaling', default_value='0.20'),
        DeclareLaunchArgument('arm_acceleration_scaling', default_value='0.20'),
        DeclareLaunchArgument('observation_mode', default_value='joint_presets'),
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python')),
        DeclareLaunchArgument('use_moveit_rviz', default_value='false'),
        DeclareLaunchArgument('use_qt_gui', default_value='true'),
        OpaqueFunction(function=partial(_launch, launch_dir=launch_dir)),
    ])

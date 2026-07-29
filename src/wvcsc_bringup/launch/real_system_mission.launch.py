# 中文说明：实机完整任务入口，默认采用 Qt 录点和 MissionManager 调度。
# 数据流为定位/录点 → Nav2 → 广域喷洒 → 停稳 → ExecuteSpray → 回 HOME。
# 本文件只做系统组装和参数传递，不复制感知、导航或机械臂状态机实现。
"""WVCSC real-hardware full-mission entry point (Qt route editor by default)."""

import os
from functools import partial
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from wvcsc_bringup.calibration_launch import (
    expand_path as _expand_path,
    latest_handeye_calibration as _latest_handeye_calibration,
    load_calibrated_mount as _load_calibrated_mount,
    load_nozzle_calibration as _load_nozzle_calibration,
    resolve_handeye_calibration as _resolve_handeye_calibration,
)
from wvcsc_bringup.path_defaults import latest_map_yaml


def _include(launch_dir, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, filename)),
        launch_arguments=(arguments or {}).items(),
    )


def _resolve_calibrations(context, *, launch_dir):
    """Load hand-eye, nozzle and camera calibrations; block if missing."""

    initial_actions = []
    # --- hand-eye calibration ---
    calibration_path = _resolve_handeye_calibration(
        LaunchConfiguration('handeye_calibration').perform(context))
    if not os.path.isfile(calibration_path):
        raise RuntimeError(
            f'hand-eye calibration is required but not found: {calibration_path}')
    xyz, rpy = _load_calibrated_mount(calibration_path)
    initial_actions.extend([
        LogInfo(msg=f'[BRINGUP] hand-eye calibration loaded: {calibration_path}'),
        SetLaunchConfiguration(
            'c10_mount_xyz', ' '.join(f'{value:.12g}' for value in xyz)),
        SetLaunchConfiguration(
            'c10_mount_rpy', ' '.join(f'{value:.12g}' for value in rpy)),
    ])

    # --- nozzle calibration ---
    nozzle_path = _expand_path(
        LaunchConfiguration('nozzle_calibration').perform(context))
    if not os.path.isfile(nozzle_path):
        raise RuntimeError(
            f'nozzle calibration is required but not found: {nozzle_path}')
    nozzle_xyz, nozzle_rpy, nozzle_plane_distance, nozzle_plane_tolerance, trim_uv = (
        _load_nozzle_calibration(nozzle_path))
    initial_actions.extend([
        LogInfo(msg=f'[BRINGUP] nozzle calibration loaded: {nozzle_path}'),
        SetLaunchConfiguration(
            'nozzle_mount_xyz',
            ' '.join(f'{value:.12g}' for value in nozzle_xyz)),
        SetLaunchConfiguration(
            'nozzle_mount_rpy',
            ' '.join(f'{value:.12g}' for value in nozzle_rpy)),
        SetLaunchConfiguration('aim_trim_u_px', str(trim_uv[0])),
        SetLaunchConfiguration('aim_trim_v_px', str(trim_uv[1])),
        SetLaunchConfiguration(
            'observation_preferred_nozzle_plane_distance_m',
            str(nozzle_plane_distance)),
        SetLaunchConfiguration(
            'observation_nozzle_plane_tolerance_m',
            str(nozzle_plane_tolerance)),
    ])
    if Path(nozzle_path).name == 'nozzle.example.yaml':
        initial_actions.append(LogInfo(
            msg='[BRINGUP][WARN] temporary nozzle calibration: '
                'spray_nozzle_link is coincident with tool0'))

    # --- C10 camera info ---
    camera_info = _expand_path(
        LaunchConfiguration('camera_info_file').perform(context))
    if os.path.isfile(camera_info):
        initial_actions.extend([
            LogInfo(msg=f'[BRINGUP] C10 CameraInfo loaded: {camera_info}'),
            SetLaunchConfiguration('camera_info_url', f'file://{camera_info}'),
        ])
    else:
        initial_actions.extend([
            LogInfo(msg=(
                '[BRINGUP][ERROR] calibrated C10 CameraInfo unavailable; '
                'mission preflight will reject automatic spraying')),
            SetLaunchConfiguration(
                'camera_info_url',
                'package://wvcsc_c10_camera/config/'
                'c10_intrinsics.yaml'),
        ])

    # --- preflight ---
    relay_config = _expand_path(
        LaunchConfiguration('relay_config_file').perform(context))
    preflight = ExecuteProcess(
        cmd=[
            LaunchConfiguration('preflight_script').perform(context),
            '--mode', 'localization',
            '--operation', 'qt_mission',
            '--camera-device', LaunchConfiguration('c10_device').perform(context),
            '--arm-device', LaunchConfiguration('serial_port').perform(context),
            '--map', LaunchConfiguration('map').perform(context),
            '--yolo-python', LaunchConfiguration(
                'yolo_python_executable').perform(context),
            '--camera-info', camera_info,
            '--handeye-calibration', calibration_path,
            '--nozzle-calibration', nozzle_path,
            '--require-nozzle-calibration', 'true',
            '--relay-config', relay_config,
        ],
        output='screen',
    )

    # --- launch chain ---
    shared_description_args = {
        'serial_port': LaunchConfiguration('serial_port'),
        'baudrate': LaunchConfiguration('baudrate'),
        'control_mode': LaunchConfiguration('control_mode'),
        'default_speed': LaunchConfiguration('default_speed'),
        'c10_mount_xyz': LaunchConfiguration('c10_mount_xyz'),
        'c10_mount_rpy': LaunchConfiguration('c10_mount_rpy'),
        'nozzle_mount_xyz': LaunchConfiguration('nozzle_mount_xyz'),
        'nozzle_mount_rpy': LaunchConfiguration('nozzle_mount_rpy'),
    }
    qt_editor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wvcsc_bringup'), 'launch',
            'nav2_qt.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_qt_gui')),
        launch_arguments={
            'use_sim_time': 'false',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'goal_pose_topic': '/manual_goal_pose',
            # Keep the recorder's admission policy in sync with the arm
            # execution mode.  The real default remains joint presets.
            'observation_mode': LaunchConfiguration('observation_mode'),
            'default_arm_spray_duration_sec': LaunchConfiguration(
                'default_arm_spray_duration_sec'),
        }.items())
    success_actions = [
        LogInfo(msg=(
            '[BRINGUP] preflight passed; starting Qt mission stack')),
        _include(launch_dir, 'real_sensors.launch.py', {
            **shared_description_args,
            'c10_device': LaunchConfiguration('c10_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }),
        _include(launch_dir, 'real_navigation.launch.py', {
            'map': LaunchConfiguration('map'),
            'start_vehicle_stack': 'false',
            'use_rviz': LaunchConfiguration('use_nav_rviz'),
            # In the full mission, RViz 2D Goal remains a Qt task-recording
            # input and must not submit an immediate Nav2 goal.
            'rviz_goal_topic': '/manual_goal_pose',
        }),
        # Keep the two RViz applications independently controlled.  In a
        # full mission the navigation display is useful for setting AMCL's
        # initial pose; the MoveIt display is normally unnecessary.
        _include(launch_dir, 'real_arm.launch.py', {
            **shared_description_args,
            'use_rviz': LaunchConfiguration('use_moveit_rviz'),
        }),
        _include(launch_dir, 'real_orchestration.launch.py', {
            **shared_description_args,
            'map': LaunchConfiguration('map'),
            'yolo_python_executable': LaunchConfiguration(
                'yolo_python_executable'),
            'vision_config_file': LaunchConfiguration('vision_config_file'),
            'use_keyboard': LaunchConfiguration('use_keyboard'),
            'arm_velocity_scaling': LaunchConfiguration(
                'arm_velocity_scaling'),
            'arm_acceleration_scaling': LaunchConfiguration(
                'arm_acceleration_scaling'),
            'observation_mode': LaunchConfiguration('observation_mode'),
            'aim_trim_u_px': LaunchConfiguration('aim_trim_u_px'),
            'aim_trim_v_px': LaunchConfiguration('aim_trim_v_px'),
            'observation_preferred_nozzle_plane_distance_m': LaunchConfiguration(
                'observation_preferred_nozzle_plane_distance_m'),
            'observation_nozzle_plane_tolerance_m': LaunchConfiguration(
                'observation_nozzle_plane_tolerance_m'),
            'relay_config_file': LaunchConfiguration('relay_config_file'),
        }),
        qt_editor,
    ]

    def after_preflight(event, _context):
        if event.returncode == 0:
            return success_actions
        return [
            LogInfo(msg=(
                '[BRINGUP][ERROR] preflight failed; no hardware stack was started')),
            EmitEvent(event=Shutdown(reason='WVCSC preflight failed')),
        ]

    return [
        *initial_actions,
        preflight,
        RegisterEventHandler(OnProcessExit(
            target_action=preflight,
            on_exit=after_preflight,
        )),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('wvcsc_bringup')
    c10_share = get_package_share_directory('wvcsc_c10_camera')
    vision_share = get_package_share_directory('wvcsc_rgb_vision')
    controller_share = get_package_share_directory('controller_pkg')
    launch_dir = os.path.join(bringup_share, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=latest_map_yaml()),
        DeclareLaunchArgument(
            'preflight_script', default_value=os.path.join(
                bringup_share, 'scripts', 'preflight_check.py')),
        DeclareLaunchArgument(
            'c10_device',
            default_value='/dev/video2'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('control_mode', default_value='pv'),
        DeclareLaunchArgument('default_speed', default_value='0.5'),
        DeclareLaunchArgument('c10_mount_xyz', default_value='-0.055 0 -0.10'),
        DeclareLaunchArgument(
            'c10_mount_rpy', default_value='0 -1.57079632679 0'),
        DeclareLaunchArgument('nozzle_mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('nozzle_mount_rpy', default_value='0 0 0'),
        DeclareLaunchArgument(
            'handeye_calibration',
            default_value='latest_real'),
        DeclareLaunchArgument(
            'nozzle_calibration',
            default_value=os.path.expanduser(
                '~/WVCSC_S2Z_UTB_ARM/src/wvcsc_perception/wvcsc_calibration/config/'
                'nozzle.example.yaml')),
        DeclareLaunchArgument(
            'relay_config_file',
            default_value=os.path.join(
                controller_share, 'config', 'fault.ini')),
        DeclareLaunchArgument('aim_trim_u_px', default_value='0.0'),
        DeclareLaunchArgument('aim_trim_v_px', default_value='0.0'),
        DeclareLaunchArgument(
            'observation_preferred_nozzle_plane_distance_m',
            default_value='1.0'),
        DeclareLaunchArgument('observation_nozzle_plane_tolerance_m',
                              default_value='0.05'),
        DeclareLaunchArgument(
            'camera_info_file',
            default_value=os.path.join(
                c10_share, 'config', 'c10_intrinsics.yaml')),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/'
                'c10_intrinsics.yaml')),
        DeclareLaunchArgument('arm_velocity_scaling', default_value='0.20'),
        DeclareLaunchArgument('arm_acceleration_scaling', default_value='0.20'),
        DeclareLaunchArgument('observation_mode', default_value='joint_presets'),
        DeclareLaunchArgument(
            'default_arm_spray_duration_sec', default_value='3.0'),
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python')),
        DeclareLaunchArgument(
            'vision_config_file',
            default_value=os.path.join(
                vision_share, 'config', 'vision_real_detect.yaml'),
            description=(
                'Perception YAML forwarded to real_orchestration. Override '
                'with vision_real.yaml to use the segment backend.')),
        # Qt route editing needs the navigation RViz for 2D Pose Estimate and
        # 2D Goal.  MoveIt RViz remains opt-in to avoid a second RViz window.
        DeclareLaunchArgument('use_nav_rviz', default_value='true'),
        DeclareLaunchArgument('use_moveit_rviz', default_value='false'),
        DeclareLaunchArgument('use_qt_gui', default_value='true'),
        DeclareLaunchArgument('use_keyboard', default_value='false'),
        OpaqueFunction(function=partial(_resolve_calibrations, launch_dir=launch_dir)),
    ])

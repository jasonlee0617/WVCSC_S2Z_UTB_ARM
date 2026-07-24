"""WVCSC real-hardware full-mission entry point (measured sites only)."""

import os
from functools import partial
import math
from pathlib import Path
import re

import yaml

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
from launch.substitutions import LaunchConfiguration


def _include(launch_dir, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, filename)),
        launch_arguments=(arguments or {}).items(),
    )


def _expand_path(path):
    return os.path.expanduser(os.path.expandvars(os.fspath(path)))


def _latest_handeye_calibration(simulation=False):
    directory = (Path.home() / 'WVCSC_S2Z_UTB_ARM' / 'src' /
                 'wvcsc_calibration' / 'config')
    prefix = 'c10_handeye_sim' if simulation else 'c10_handeye'
    pattern = re.compile(
        rf'^{re.escape(prefix)}_(\d{{8}}_\d{{6}})\.calib$')
    candidates = []
    for path in directory.glob(f'{prefix}_*.calib'):
        match = pattern.fullmatch(path.name)
        if match:
            candidates.append((match.group(1), path.name, path))
    if not candidates:
        role = 'simulation' if simulation else 'real'
        raise RuntimeError(
            f'no timestamped {role} C10 hand-eye calibration in {directory}')
    return str(max(candidates, key=lambda item: (item[0], item[1]))[2])


def _resolve_handeye_calibration(value, *, simulation=False):
    value = os.fspath(value)
    if value in ('', 'latest', 'latest_real'):
        return _latest_handeye_calibration(simulation=simulation)
    if value == 'latest_sim':
        return _latest_handeye_calibration(simulation=True)
    return _expand_path(value)


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _transpose(matrix):
    return tuple(zip(*matrix))


def _multiply(left, right):
    return tuple(tuple(
        sum(left[row][index] * right[index][column] for index in range(3))
        for column in range(3)) for row in range(3))


def _quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not 0.95 <= norm <= 1.05:
        raise RuntimeError('hand-eye quaternion is not normalized')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)),
    )


def _matrix_rpy(matrix):
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) < 1.0e-8:
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    else:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    return roll, pitch, yaw


def _load_calibrated_mount(path):
    with open(_expand_path(path), encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    # Accept both easy_handeye2's raw ``parameters``/``transform`` file and
    # the validated deployment file with a top-level ``calibration`` mapping.
    # Keep this small parser local: wvcsc_calibration already depends on
    # wvcsc_bringup for its calibration launch, so importing it here would
    # create a colcon dependency cycle.
    if 'calibration' in data:
        calibration = data.get('calibration')
        if not isinstance(calibration, dict):
            raise RuntimeError('hand-eye calibration must be a YAML mapping')
        if calibration.get('type') != 'eye_in_hand':
            raise RuntimeError('hand-eye calibration type must be eye_in_hand')
    else:
        parameters = data.get('parameters', {})
        if (parameters.get('calibration_type') != 'eye_in_hand' or
                parameters.get('robot_base_frame') != 'alicia_base_link' or
                parameters.get('robot_effector_frame') != 'tool0' or
                parameters.get('tracking_base_frame') !=
                'camera_color_optical_frame'):
            raise RuntimeError(
                'raw hand-eye calibration must describe alicia_base_link, '
                'tool0 and camera_color_optical_frame')
        transform = data.get('transform', {})
        calibration = {
            'parent_frame': 'tool0',
            'child_frame': 'camera_color_optical_frame',
            'translation': transform.get('translation', {}),
            'rotation': transform.get('rotation', {}),
        }
    if (calibration.get('parent_frame') != 'tool0' or
            calibration.get('child_frame') != 'camera_color_optical_frame'):
        raise RuntimeError(
            'hand-eye calibration must describe tool0 -> '
            'camera_color_optical_frame')
    translation = calibration.get('translation', {})
    rotation = calibration.get('rotation', {})
    xyz = tuple(float(translation[key]) for key in ('x', 'y', 'z'))
    quaternion = tuple(float(rotation[key]) for key in ('x', 'y', 'z', 'w'))
    if not all(math.isfinite(value) for value in (*xyz, *quaternion)):
        raise RuntimeError('hand-eye calibration contains non-finite values')
    tool_to_optical = _quaternion_matrix(*quaternion)
    link_to_optical = _rpy_matrix(-math.pi / 2.0, 0.0, -math.pi / 2.0)
    tool_to_link = _multiply(tool_to_optical, _transpose(link_to_optical))
    return xyz, _matrix_rpy(tool_to_link)


def _load_nozzle_calibration(path):
    with open(_expand_path(path), encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if int(data.get('schema_version', 0)) != 1:
        raise RuntimeError('nozzle calibration schema_version must be 1')
    if (data.get('parent_frame') != 'tool0' or
            data.get('child_frame') != 'spray_nozzle_link'):
        raise RuntimeError(
            'nozzle calibration must describe tool0 -> spray_nozzle_link')
    translation = data.get('translation', {})
    rotation = data.get('rotation', {})
    xyz = tuple(float(translation[key]) for key in ('x', 'y', 'z'))
    quaternion = tuple(float(rotation[key]) for key in ('x', 'y', 'z', 'w'))
    working_distance = float(data['working_distance_m'])
    tolerance = float(data['working_distance_tolerance_m'])
    trim = data.get('pixel_trim', {})
    trim_uv = (float(trim.get('u', 0.0)), float(trim.get('v', 0.0)))
    values = (*xyz, *quaternion, working_distance, tolerance, *trim_uv)
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError('nozzle calibration contains non-finite values')
    if math.sqrt(sum(value * value for value in xyz)) > 0.30:
        raise RuntimeError('nozzle translation exceeds the 0.30 m sanity limit')
    return (
        xyz,
        _matrix_rpy(_quaternion_matrix(*quaternion)),
        working_distance,
        tolerance,
        trim_uv,
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
    nozzle_xyz, nozzle_rpy, aim_range, aim_tolerance, trim_uv = (
        _load_nozzle_calibration(nozzle_path))
    initial_actions.extend([
        LogInfo(msg=f'[BRINGUP] nozzle calibration loaded: {nozzle_path}'),
        SetLaunchConfiguration(
            'nozzle_mount_xyz',
            ' '.join(f'{value:.12g}' for value in nozzle_xyz)),
        SetLaunchConfiguration(
            'nozzle_mount_rpy',
            ' '.join(f'{value:.12g}' for value in nozzle_rpy)),
        SetLaunchConfiguration('aim_fixed_range_m', str(aim_range)),
        SetLaunchConfiguration('aim_range_tolerance_m', str(aim_tolerance)),
        SetLaunchConfiguration('aim_trim_u_px', str(trim_uv[0])),
        SetLaunchConfiguration('aim_trim_v_px', str(trim_uv[1])),
    ])

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
            '--operation', 'field_route',
            '--mission-file', LaunchConfiguration('mission_file').perform(context),
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
    success_actions = [
        LogInfo(msg='[BRINGUP] preflight passed; starting full mission stack'),
        _include(launch_dir, 'real_sensors.launch.py', {
            **shared_description_args,
            'c10_device': LaunchConfiguration('c10_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }),
        _include(launch_dir, 'real_navigation.launch.py', {
            'map': LaunchConfiguration('map'),
            'start_vehicle_stack': 'false',
            'use_rviz': LaunchConfiguration('use_nav_rviz'),
        }),
        _include(launch_dir, 'real_arm.launch.py', shared_description_args),
        _include(launch_dir, 'real_orchestration.launch.py', {
            **shared_description_args,
            'map': LaunchConfiguration('map'),
            'mission_file': LaunchConfiguration('mission_file'),
            'yolo_python_executable': LaunchConfiguration(
                'yolo_python_executable'),
            'use_keyboard': LaunchConfiguration('use_keyboard'),
            'arm_velocity_scaling': LaunchConfiguration(
                'arm_velocity_scaling'),
            'arm_acceleration_scaling': LaunchConfiguration(
                'arm_acceleration_scaling'),
            'observation_mode': LaunchConfiguration('observation_mode'),
            'aim_fixed_range_m': LaunchConfiguration('aim_fixed_range_m'),
            'aim_range_tolerance_m': LaunchConfiguration(
                'aim_range_tolerance_m'),
            'aim_trim_u_px': LaunchConfiguration('aim_trim_u_px'),
            'aim_trim_v_px': LaunchConfiguration('aim_trim_v_px'),
            'relay_config_file': LaunchConfiguration('relay_config_file'),
        }),
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
    controller_share = get_package_share_directory('controller_pkg')
    launch_dir = os.path.join(bringup_share, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_file', default_value=os.path.expanduser(
                '~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/field_route_corn.yaml')),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(
                os.path.expanduser('~/WVCSC_S2Z_UTB_ARM/src'),
                'wvcsc_bringup', 'maps', 'orchard.yaml')),
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
                '~/.ros/wvcsc_calibration/nozzle.yaml')),
        DeclareLaunchArgument(
            'relay_config_file',
            default_value=os.path.join(
                controller_share, 'config', 'fault.ini')),
        DeclareLaunchArgument('aim_fixed_range_m', default_value='1.0'),
        DeclareLaunchArgument('aim_range_tolerance_m', default_value='0.05'),
        DeclareLaunchArgument('aim_trim_u_px', default_value='0.0'),
        DeclareLaunchArgument('aim_trim_v_px', default_value='0.0'),
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
            'yolo_python_executable',
            default_value=os.path.expanduser(
                '~/venvs/wvcsc_yolo_ros/bin/python')),
        DeclareLaunchArgument('use_nav_rviz', default_value='false'),
        DeclareLaunchArgument('use_keyboard', default_value='false'),
        OpaqueFunction(function=partial(_resolve_calibrations, launch_dir=launch_dir)),
    ])

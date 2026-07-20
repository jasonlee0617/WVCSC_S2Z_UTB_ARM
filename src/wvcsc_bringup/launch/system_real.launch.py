"""Mutually exclusive WVCSC real-hardware system entry point."""

import os
from functools import partial
import math

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
    with open(os.path.expanduser(path), encoding='utf-8') as stream:
        calibration = (yaml.safe_load(stream) or {}).get('calibration', {})
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
    """Validate tool0->spray_nozzle_link and return launch-ready values."""
    with open(os.path.expanduser(path), encoding='utf-8') as stream:
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
    if (working_distance <= 0.0 or tolerance <= 0.0 or
            abs(working_distance - 1.0) > 1.0e-6 or
            abs(tolerance - 0.05) > 1.0e-6):
        raise RuntimeError(
            'nozzle calibration must use working_distance=1.0 and '
            'tolerance=0.05 m')
    if math.sqrt(sum(value * value for value in xyz)) > 0.30:
        raise RuntimeError('nozzle translation exceeds the 0.30 m sanity limit')
    return (
        xyz,
        _matrix_rpy(_quaternion_matrix(*quaternion)),
        working_distance,
        tolerance,
        trim_uv,
    )


def _select_mode(context, *, launch_dir):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    if mode not in {'localization', 'mapping'}:
        raise RuntimeError('mode must be localization or mapping')
    operation = LaunchConfiguration('operation').perform(context).strip().lower()
    if operation not in {'survey', 'mission'}:
        raise RuntimeError('operation must be survey or mission')
    mission_source = LaunchConfiguration(
        'mission_source').perform(context).strip().lower()
    if mission_source not in {'measured', 'uav'}:
        raise RuntimeError('mission_source must be measured or uav')

    initial_actions = []
    calibration_path = LaunchConfiguration('handeye_calibration').perform(context)
    use_calibration = LaunchConfiguration(
        'use_handeye_calibration').perform(context).lower() == 'true'
    mission_mode = mode == 'localization' and operation == 'mission'
    if mission_mode and not use_calibration:
        raise RuntimeError(
            'use_handeye_calibration cannot be disabled in mission mode')
    calibrated = (
        mission_mode and use_calibration and
        os.path.isfile(os.path.expanduser(calibration_path)))
    if calibrated:
        xyz, rpy = _load_calibrated_mount(calibration_path)
        initial_actions.extend([
            LogInfo(msg=f'[BRINGUP] using hand-eye calibration: {calibration_path}'),
            SetLaunchConfiguration(
                'c10_mount_xyz', ' '.join(f'{value:.12g}' for value in xyz)),
            SetLaunchConfiguration(
                'c10_mount_rpy', ' '.join(f'{value:.12g}' for value in rpy)),
        ])
    elif mission_mode:
        initial_actions.append(LogInfo(msg=(
            '[BRINGUP][ERROR] required hand-eye calibration is unavailable: '
            f'{calibration_path}')))

    nozzle_path = LaunchConfiguration('nozzle_calibration').perform(context)
    require_nozzle = LaunchConfiguration(
        'require_nozzle_calibration').perform(context).lower() == 'true'
    if mission_mode and not require_nozzle:
        raise RuntimeError(
            'require_nozzle_calibration cannot be disabled in mission mode')
    nozzle_available = os.path.isfile(os.path.expanduser(nozzle_path))
    if mission_mode and nozzle_available:
        nozzle_xyz, nozzle_rpy, aim_range, aim_tolerance, trim_uv = (
            _load_nozzle_calibration(nozzle_path))
        initial_actions.extend([
            LogInfo(msg=f'[BRINGUP] using nozzle calibration: {nozzle_path}'),
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
    elif mission_mode and require_nozzle:
        initial_actions.append(LogInfo(msg=(
            '[BRINGUP][ERROR] required nozzle calibration is unavailable: '
            f'{nozzle_path}')))

    if mode == 'localization':
        camera_info = os.path.expanduser(
            LaunchConfiguration('camera_info_file').perform(context))
        if os.path.isfile(camera_info):
            initial_actions.extend([
                LogInfo(msg=f'[BRINGUP] using C10 CameraInfo: {camera_info}'),
                SetLaunchConfiguration(
                    'camera_info_url', f'file://{camera_info}'),
            ])
        else:
            initial_actions.extend([
                LogInfo(msg=(
                    '[BRINGUP][ERROR] calibrated C10 CameraInfo unavailable; '
                    'mission preflight will reject automatic spraying')),
                SetLaunchConfiguration(
                    'camera_info_url',
                    'package://wvcsc_c10_camera/config/'
                    'c10_reference_calibration.yaml'),
            ])

    preflight = ExecuteProcess(
        cmd=[
            LaunchConfiguration('preflight_script').perform(context),
            '--mode', mode,
            '--operation', operation,
            '--mission-source', mission_source,
            '--mission-file', LaunchConfiguration('mission_file').perform(context),
            '--camera-device', LaunchConfiguration('c10_device').perform(context),
            '--arm-device', LaunchConfiguration('serial_port').perform(context),
            '--map', LaunchConfiguration('map').perform(context),
            '--yolo-python', LaunchConfiguration(
                'yolo_python_executable').perform(context),
            '--camera-info', LaunchConfiguration(
                'camera_info_file').perform(context),
            '--handeye-calibration', calibration_path,
            '--nozzle-calibration', nozzle_path,
            '--require-nozzle-calibration',
            str(require_nozzle).lower(),
        ],
        output='screen',
    )

    if mode == 'mapping':
        success_actions = [
            LogInfo(msg='[BRINGUP] preflight passed; starting mapping-only stack'),
            _include(launch_dir, 'real_cartographer.launch.py'),
        ]
    else:
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
            LogInfo(msg=(
                '[BRINGUP] preflight passed; starting localization '
                f'operation={operation}')),
            _include(launch_dir, 'real_sensors.launch.py', {
                **shared_description_args,
                'c10_device': LaunchConfiguration('c10_device'),
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }),
            _include(launch_dir, 'real_navigation.launch.py', {
                'map': LaunchConfiguration('map'),
                'use_rviz': LaunchConfiguration('use_nav_rviz'),
            }),
        ]
        if operation == 'mission':
            success_actions.extend([
                _include(
                    launch_dir, 'real_arm.launch.py', shared_description_args),
                _include(launch_dir, 'real_orchestration.launch.py', {
                    **shared_description_args,
                    'map': LaunchConfiguration('map'),
                    'mission_source': LaunchConfiguration('mission_source'),
                    'mission_file': LaunchConfiguration('mission_file'),
                    'yolo_python_executable': LaunchConfiguration(
                        'yolo_python_executable'),
                    'use_keyboard': LaunchConfiguration('use_keyboard'),
                    'arm_velocity_scaling': LaunchConfiguration(
                        'arm_velocity_scaling'),
                    'arm_acceleration_scaling': LaunchConfiguration(
                        'arm_acceleration_scaling'),
                    'aim_fixed_range_m': LaunchConfiguration(
                        'aim_fixed_range_m'),
                    'aim_range_tolerance_m': LaunchConfiguration(
                        'aim_range_tolerance_m'),
                    'aim_trim_u_px': LaunchConfiguration('aim_trim_u_px'),
                    'aim_trim_v_px': LaunchConfiguration('aim_trim_v_px'),
                }),
            ])

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
    navigation_share = get_package_share_directory('my_navigation2')
    launch_dir = os.path.join(bringup_share, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description='Exactly one of: localization, mapping.'),
        DeclareLaunchArgument(
            'operation', default_value='mission',
            description='Localization operation: survey or mission.'),
        DeclareLaunchArgument(
            'mission_source', default_value='measured',
            description='Mission source: measured or uav.'),
        DeclareLaunchArgument(
            'mission_file', default_value=os.path.expanduser(
                '~/.ros/wvcsc_sites/corn_site.yaml')),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(
                navigation_share, 'maps', 'map_new.yaml')),
        DeclareLaunchArgument(
            'preflight_script', default_value=os.path.join(
                bringup_share, 'scripts', 'preflight_check.py')),
        DeclareLaunchArgument(
            'c10_device',
            default_value='/dev/v4l/by-id/usb-Synria_C10-video-index0'),
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
            default_value=os.path.expanduser(
                '~/.ros/wvcsc_calibration/c10_handeye.yaml')),
        DeclareLaunchArgument('use_handeye_calibration', default_value='true'),
        DeclareLaunchArgument(
            'nozzle_calibration',
            default_value=os.path.expanduser(
                '~/.ros/wvcsc_calibration/nozzle.yaml')),
        DeclareLaunchArgument(
            'require_nozzle_calibration', default_value='true'),
        DeclareLaunchArgument('aim_fixed_range_m', default_value='1.0'),
        DeclareLaunchArgument('aim_range_tolerance_m', default_value='0.05'),
        DeclareLaunchArgument('aim_trim_u_px', default_value='0.0'),
        DeclareLaunchArgument('aim_trim_v_px', default_value='0.0'),
        DeclareLaunchArgument(
            'camera_info_file',
            default_value=os.path.expanduser('~/.ros/camera_info/c10.yaml')),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/'
                'c10_reference_calibration.yaml')),
        DeclareLaunchArgument('arm_velocity_scaling', default_value='0.20'),
        DeclareLaunchArgument('arm_acceleration_scaling', default_value='0.20'),
        DeclareLaunchArgument(
            'yolo_python_executable',
            default_value='/home/robot/venvs/wvcsc_yolo_ros/bin/python'),
        DeclareLaunchArgument('use_nav_rviz', default_value='false'),
        DeclareLaunchArgument('use_keyboard', default_value='false'),
        OpaqueFunction(function=partial(_select_mode, launch_dir=launch_dir)),
    ])

#!/usr/bin/env python3
"""Fail-fast checks for WVCSC mapping and Qt-created real missions."""

import argparse
import configparser
import os
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)


COMMON_PACKAGES = (
    'can_bridge', 'wtb_car_driver', 'lslidar_driver',
    'yesense_interface', 'yesense_std_ros2',
    'robot_localization', 'pointcloud_to_laserscan',
    'robot_state_publisher', 'xacro',
)
# fdilink_ahrs is intentionally not checked: its launch entrypoint is a
# commented rollback path, while Yesense is the only active real IMU driver.
LOCALIZATION_PACKAGES = (
    'nav2_bringup', 'usb_cam', 'wvcsc_c10_camera',
    'wvcsc_description',
)
MISSION_PACKAGES = (
    'controller_manager', 'rclcpp_components',
    'alicia_m_bringup', 'alicia_m_driver', 'alicia_m_moveit_config',
    'moveit_ros_move_group', 'moveit_servo', 'pymoveit2',
    'trajectory_retime_server', 'wvcsc_interfaces', 'wvcsc_rgb_vision',
    'wvcsc_visual_servo', 'wvcsc_arm_task', 'wvcsc_mission_manager',
    'controller_pkg',
)
MAPPING_PACKAGES = (
    'cartographer_ros', 'my_cartographer', 'joint_state_publisher', 'rviz2',
)
REAL_MODELS = ('yolov8s_real.pt', 'yolov8s_seg_real.pt')
REAL_MODEL_CONTRACTS = (
    ('yolov8s_real.pt', 'detect', {0: 'tree'}),
    ('yolov8s_seg_real.pt', 'segment', {0: 'disease_leaf'}),
)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', required=True, choices=('localization', 'mapping'))
    parser.add_argument(
        '--operation', default='qt_mission', choices=('survey', 'qt_mission'))
    parser.add_argument('--camera-device', default='')
    parser.add_argument('--arm-device', default='')
    parser.add_argument('--map', default='')
    parser.add_argument('--yolo-python', default='')
    parser.add_argument('--camera-info', default='')
    parser.add_argument('--handeye-calibration', default='')
    parser.add_argument('--nozzle-calibration', default='')
    parser.add_argument('--relay-config', default='')
    parser.add_argument(
        '--require-nozzle-calibration', default='true',
        choices=('true', 'false'))
    return parser.parse_args()


def _exists(label, path, failures):
    candidate = Path(path).expanduser()
    if candidate.exists():
        print(f'  [OK]   {label}: {candidate}')
        return
    failures.append(f'{label} not found: {candidate}')
    print(f'  [FAIL] {label}: {candidate}')


def _packages(names, failures):
    for name in names:
        try:
            get_package_prefix(name)
            print(f'  [OK]   ROS package: {name}')
        except PackageNotFoundError:
            failures.append(f'ROS package not found: {name}')
            print(f'  [FAIL] ROS package: {name}')


def _yolo_runtime(interpreter, failures):
    executable = Path(interpreter).expanduser()
    if not executable.is_file() or not os.access(executable, os.X_OK):
        failures.append(f'YOLO Python is not executable: {executable}')
        print(f'  [FAIL] YOLO Python: {executable}')
        return
    environment = os.environ.copy()
    environment['PYTHONNOUSERSITE'] = '1'
    environment['YOLO_CONFIG_DIR'] = '/tmp/wvcsc_ultralytics'
    result = subprocess.run(
        [str(executable), '-c',
         'import cv_bridge, rclpy, torch, ultralytics; print("runtime ok")'],
        env=environment, capture_output=True, text=True, timeout=30, check=False)
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()
        failures.append(f'YOLO runtime import failed: {detail}')
        print(f'  [FAIL] YOLO runtime: {detail}')
    else:
        print(f'  [OK]   YOLO runtime: {executable}')


def _yolo_contracts(interpreter, model_dir, failures):
    """Load real weights before hardware startup and enforce exact contracts."""
    executable = Path(interpreter).expanduser()
    if not executable.is_file() or not os.access(executable, os.X_OK):
        return
    if not all((model_dir / name).is_file() for name, _task, _names in
               REAL_MODEL_CONTRACTS):
        return
    checker = r'''
import json
import sys
from ultralytics import YOLO

contracts = json.loads(sys.argv[1])
for path, expected_task, expected_names in contracts:
    model = YOLO(path)
    names = model.names
    actual_names = ({int(key): str(value) for key, value in names.items()}
                    if isinstance(names, dict)
                    else {index: str(value) for index, value in enumerate(names)})
    expected_names = {int(key): str(value)
                      for key, value in expected_names.items()}
    if model.task != expected_task or actual_names != expected_names:
        raise SystemExit(
            f'{path}: expected task={expected_task}, names={expected_names}; '
            f'found task={model.task}, names={actual_names}')
print('model contracts ok')
'''
    import json
    contracts = [
        [str(model_dir / name), task, names]
        for name, task, names in REAL_MODEL_CONTRACTS
    ]
    environment = os.environ.copy()
    environment['PYTHONNOUSERSITE'] = '1'
    environment['YOLO_CONFIG_DIR'] = '/tmp/wvcsc_ultralytics'
    result = subprocess.run(
        [str(executable), '-c', checker, json.dumps(contracts)],
        env=environment, capture_output=True, text=True, timeout=60,
        check=False)
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()
        failures.append(f'real YOLO model contract failed: {detail}')
        print(f'  [FAIL] real YOLO contracts: {detail}')
    else:
        print('  [OK]   real YOLO contracts: tree + disease_leaf')


def _calibration_checks(args, failures):
    """Mission mode is fail-closed for all geometry used by spraying."""
    _exists('C10 CameraInfo', args.camera_info, failures)
    _exists('C10 hand-eye calibration', args.handeye_calibration, failures)
    if args.require_nozzle_calibration == 'true':
        _exists('spray nozzle calibration', args.nozzle_calibration, failures)


def _relay_config(path, failures):
    config_path = Path(path).expanduser()
    _exists('relay configuration', config_path, failures)
    if not config_path.is_file():
        return
    parser = configparser.ConfigParser()
    try:
        parser.read(config_path, encoding='utf-8')
        section = parser['serial']
        port = section['PortName'].strip()
        baudrate = section.getint('BaudRate')
        address = section.getint('Address')
        timeout = section.getfloat('Timeout')
    except (KeyError, ValueError, configparser.Error) as error:
        failures.append(f'invalid relay configuration: {error}')
        print(f'  [FAIL] relay configuration: {error}')
        return
    if (not port or baudrate <= 0 or not 1 <= address <= 255 or
            timeout <= 0.0):
        failures.append('relay configuration values are out of range')
        print('  [FAIL] relay configuration values are out of range')
        return
    _exists('relay serial', port, failures)


def main():
    args = _arguments()
    failures = []
    print(
        f'=== WVCSC preflight: mode={args.mode} '
        f'operation={args.operation} ===')
    _packages(COMMON_PACKAGES, failures)

    if args.mode == 'mapping':
        _packages(MAPPING_PACKAGES, failures)
    else:
        _packages(LOCALIZATION_PACKAGES, failures)
        _exists('C10 camera', args.camera_device, failures)
        _exists('map YAML', args.map, failures)
        if args.operation == 'qt_mission':
            _packages(MISSION_PACKAGES, failures)
            _exists('Alicia-M serial', args.arm_device, failures)
            _yolo_runtime(args.yolo_python, failures)
            try:
                model_dir = Path(
                    get_package_share_directory('wvcsc_rgb_vision')) / 'models'
            except PackageNotFoundError:
                model_dir = Path('/package-not-found')
            for model in REAL_MODELS:
                _exists(f'real YOLO weight {model}', model_dir / model, failures)
            _yolo_contracts(args.yolo_python, model_dir, failures)
            _calibration_checks(args, failures)
            _relay_config(args.relay_config, failures)
            # The operator creates the route in Qt only after AMCL is ready.
            # There is deliberately no file-backed route contract at launch.

    if failures:
        print('\n[ABORT] Real bringup prerequisites failed:')
        for failure in failures:
            print(f'  - {failure}')
        return 1
    print('\n[READY] Preflight passed; launch may start the selected mode.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

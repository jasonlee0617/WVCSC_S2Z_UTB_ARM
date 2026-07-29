# 中文说明：bringup 启动文件共用的纯标定读取工具。
# 它只解析路径和标定内容，不创建 ROS 节点、不发布 TF，调用方保留启动时序控制权。
# 实机启动必须继续 fail-closed，缺少有效时间戳标定时不能假装使用默认值。
"""Shared pure calibration loading for real WVCSC launch files.

The calibration package launches Bringup's arm stack, so these helpers stay in
``wvcsc_bringup`` to avoid a package dependency cycle.  They deliberately do
not create ROS entities or launch actions; callers retain their existing
arguments, preflight sequence, and launch topology.
"""

import math
import os
from pathlib import Path
import re

import yaml


def expand_path(path):
    """Expand the same user and environment variables accepted by launch args."""
    return os.path.expanduser(os.path.expandvars(os.fspath(path)))


def latest_handeye_calibration(simulation=False):
    """Return the newest timestamped real or simulated C10 hand-eye file."""
    directory = (Path.home() / 'WVCSC_S2Z_UTB_ARM' / 'src' /
                 'wvcsc_perception' / 'wvcsc_calibration' / 'config')
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


def resolve_handeye_calibration(value, *, simulation=False):
    """Resolve a launch argument without changing its existing aliases."""
    value = os.fspath(value)
    if value in ('', 'latest', 'latest_real'):
        return latest_handeye_calibration(simulation=simulation)
    if value == 'latest_sim':
        return latest_handeye_calibration(simulation=True)
    return expand_path(value)


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


def load_calibrated_mount(path):
    """Load tool0-to-C10 calibration and convert it to the URDF C10 link RPY."""
    with open(expand_path(path), encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
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


def load_nozzle_calibration(path):
    """Load the strict tool0-to-spray-nozzle deployment calibration contract."""
    with open(expand_path(path), encoding='utf-8') as stream:
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

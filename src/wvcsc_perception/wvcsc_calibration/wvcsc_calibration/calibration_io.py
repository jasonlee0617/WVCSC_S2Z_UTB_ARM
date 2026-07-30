"""Validate and persist isolated native easy_handeye2 calibrations."""

import argparse
import math
import os
from pathlib import Path
import tempfile

import yaml

from wvcsc_bringup.handeye_calibration_paths import (
    calibration_config_dir,
    calibration_root_dir,
    expanded_path,
    latest_calibration_path,
    resolve_handeye_calibration,
    timestamped_calibration_paths,
)

def _finite(mapping, keys):
    values = []
    for key in keys:
        try:
            value = float(mapping[key])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'missing or invalid transform.{key}') from error
        if not math.isfinite(value):
            raise ValueError(f'transform.{key} must be finite')
        values.append(value)
    return values


def normalized_calibration(data):
    """Read either legacy normalized YAML or native easy_handeye2 data."""
    if not isinstance(data, dict):
        raise ValueError('calibration must be a YAML mapping')
    if 'calibration' in data and 'parameters' not in data:
        calibration = data.get('calibration')
        if not isinstance(calibration, dict):
            raise ValueError('calibration must be a YAML mapping')
        if (calibration.get('type') != 'eye_in_hand' or
                calibration.get('parent_frame') != 'tool0' or
                calibration.get('child_frame') != 'camera_color_optical_frame'):
            raise ValueError(
                'normalized calibration must describe tool0 -> '
                'camera_color_optical_frame')
        translation = _finite(calibration.get('translation', {}), ('x', 'y', 'z'))
        rotation = _finite(calibration.get('rotation', {}), ('x', 'y', 'z', 'w'))
    else:
        parameters = data.get('parameters', {})
        if parameters.get('calibration_type') != 'eye_in_hand':
            raise ValueError('calibration_type must be eye_in_hand')
        if parameters.get('robot_base_frame') != 'alicia_base_link':
            raise ValueError('robot_base_frame must be alicia_base_link')
        if parameters.get('robot_effector_frame') != 'tool0':
            raise ValueError('robot_effector_frame must be tool0')
        if parameters.get('tracking_base_frame') != 'camera_color_optical_frame':
            raise ValueError('tracking_base_frame must be camera_color_optical_frame')
        transform = data.get('transform', {})
        translation = _finite(transform.get('translation', {}), ('x', 'y', 'z'))
        rotation = _finite(transform.get('rotation', {}), ('x', 'y', 'z', 'w'))
    norm = math.sqrt(sum(value * value for value in rotation))
    if not 0.95 <= norm <= 1.05:
        raise ValueError(f'quaternion norm is invalid: {norm:.6f}')
    if math.sqrt(sum(value * value for value in translation)) > 0.50:
        raise ValueError('camera translation exceeds the 0.50 m bracket sanity limit')
    return {
        'calibration': {
            'type': 'eye_in_hand',
            'parent_frame': 'tool0',
            'child_frame': 'camera_color_optical_frame',
            'translation': dict(zip(('x', 'y', 'z'), translation)),
            'rotation': dict(zip(
                ('x', 'y', 'z', 'w'), (value / norm for value in rotation))),
            'marker': {'dictionary': 'DICT_5X5_250', 'id': 1, 'size_m': 0.070},
        },
    }


def _easy_handeye_calibration(transform):
    """Build and validate the native easy_handeye2 representation."""
    translation, rotation = transform
    source = {
        'parameters': {
            'name': 'wvcsc_c10',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'alicia_base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'calibration_aruco',
            'freehand_robot_movement': True,
            'move_group_namespace': '/',
            'move_group': 'arm',
        },
        'transform': {
            'translation': dict(zip(('x', 'y', 'z'), translation)),
            'rotation': dict(zip(('x', 'y', 'z', 'w'), rotation)),
        },
    }
    normalized_calibration(source)
    return source


def native_calibration(data):
    """Convert a validated legacy or native mapping to native easy_handeye2."""
    calibration = normalized_calibration(data)['calibration']
    return _easy_handeye_calibration((
        tuple(calibration['translation'][key] for key in ('x', 'y', 'z')),
        tuple(calibration['rotation'][key] for key in ('x', 'y', 'z', 'w'))))


def export_calibration(input_path, output_path):
    source = resolve_handeye_calibration(input_path)
    with source.open(encoding='utf-8') as stream:
        calibration = native_calibration(yaml.safe_load(stream))
    return _write_yaml_payloads_atomically([(calibration, output_path)])[0]


def write_calibration(transform, output_path):
    """Atomically write one native easy_handeye2 ``.calib`` result."""
    return _write_yaml_payloads_atomically([
        (_easy_handeye_calibration(transform), output_path)])[0]


def write_calibration_outputs(
        transform, calibration_output_path, samples_yaml, samples_output_path):
    """Atomically persist the successful native calibration and sample archive."""
    return _write_payloads_atomically([
        (yaml.safe_dump(_easy_handeye_calibration(transform), sort_keys=False)
         .encode('utf-8'), calibration_output_path),
        (str(samples_yaml).encode('utf-8'), samples_output_path),
    ])


def write_sample_archive(samples_yaml, output_path):
    """Persist valid partial samples after a non-cancelled failed run."""
    return _write_payloads_atomically([
        (str(samples_yaml).encode('utf-8'), output_path)])[0]


def _stage_bytes(destination, payload):
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{destination.name}.', suffix='.tmp', dir=str(destination.parent))
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        return temporary
    except Exception:
        if temporary.exists():
            temporary.unlink()
        raise


def _write_yaml_payloads_atomically(payloads):
    return _write_payloads_atomically([
        (yaml.safe_dump(document, sort_keys=False).encode('utf-8'), output_path)
        for document, output_path in payloads])


def _write_payloads_atomically(payloads):
    """Stage all outputs before replacing them, restoring on partial failure."""
    staged = []
    backups = []
    installed = []
    destinations = []
    try:
        for payload, output_path in payloads:
            destination = expanded_path(output_path)
            if destination in destinations:
                raise ValueError(f'duplicate calibration output path: {destination}')
            destinations.append(destination)
            staged.append((destination, _stage_bytes(destination, payload)))
        for destination, _temporary in staged:
            backup = (_stage_bytes(destination, destination.read_bytes())
                      if destination.exists() else None)
            backups.append((destination, backup))
        for destination, temporary in staged:
            os.replace(temporary, destination)
            installed.append(destination)
        return tuple(destination for destination, _temporary in staged)
    except Exception:
        backup_by_destination = dict(backups)
        for destination in reversed(installed):
            backup = backup_by_destination.get(destination)
            if backup is not None and backup.exists():
                os.replace(backup, destination)
            elif destination.exists():
                destination.unlink()
        raise
    finally:
        for _destination, temporary in staged:
            if temporary.exists():
                temporary.unlink()
        for _destination, backup in backups:
            if backup is not None and backup.exists():
                backup.unlink()


def main():
    parser = argparse.ArgumentParser(
        description='Validate and export a native easy_handeye2 C10 calibration.')
    parser.add_argument('--input', default='latest_real')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = export_calibration(args.input, args.output)
    print(f'Native easy_handeye2 hand-eye calibration written to {output}')

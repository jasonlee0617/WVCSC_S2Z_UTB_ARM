"""Validate and export easy_handeye2 output to the WVCSC deployment file."""

import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import re
import tempfile

import yaml


_TIMESTAMPED_CALIBRATION = re.compile(
    r'^c10_handeye(?P<sim>_sim)?_(?P<stamp>\d{8}_\d{6})\.(?P<kind>calib|yaml)$')


def expanded_path(path):
    """Expand the portable ``$HOME`` and ``~`` forms used by calibration YAML."""
    return Path(os.path.expanduser(os.path.expandvars(os.fspath(path))))


def calibration_config_dir():
    """Return the source-workspace calibration directory used at runtime."""
    return (Path.home() / 'WVCSC_S2Z_UTB_ARM' / 'src' /
            'wvcsc_calibration' / 'config')


def timestamped_calibration_paths(output_dir=None, *, simulation=False,
                                  timestamp=None):
    """Return matching native and normalized paths for one calibration run."""
    directory = expanded_path(output_dir or calibration_config_dir())
    stamp = timestamp or datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
    if not re.fullmatch(r'\d{8}_\d{6}', stamp):
        raise ValueError('calibration timestamp must be YYYYMMDD_HHMMSS')
    prefix = 'c10_handeye_sim' if simulation else 'c10_handeye'
    stem = f'{prefix}_{stamp}'
    return directory / f'{stem}.calib', directory / f'{stem}.yaml'


def latest_calibration_path(directory=None, *, simulation=False, kind='calib'):
    """Select the newest timestamped real or simulation calibration file."""
    if kind not in ('calib', 'yaml'):
        raise ValueError("calibration kind must be 'calib' or 'yaml'")
    directory = expanded_path(directory or calibration_config_dir())
    prefix = 'c10_handeye_sim' if simulation else 'c10_handeye'
    candidates = []
    for path in directory.glob(f'{prefix}_*.{kind}'):
        match = _TIMESTAMPED_CALIBRATION.fullmatch(path.name)
        if match is None or bool(match.group('sim')) != bool(simulation):
            continue
        candidates.append((match.group('stamp'), path.name, path))
    if not candidates:
        role = 'simulation' if simulation else 'real'
        raise FileNotFoundError(
            f'no timestamped {role} C10 hand-eye {kind} file in {directory}')
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


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
    """Return a strict eye-in-hand transform or raise before deployment."""
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
                'rotation': dict(zip(('x', 'y', 'z', 'w'),
                                     (value / norm for value in rotation))),
                'marker': calibration.get('marker', {
                    'dictionary': 'DICT_5X5_250', 'id': 1, 'size_m': 0.070}),
            },
        }
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
    rotation = [value / norm for value in rotation]
    return {
        'calibration': {
            'type': 'eye_in_hand',
            'parent_frame': 'tool0',
            'child_frame': 'camera_color_optical_frame',
            'translation': dict(zip(('x', 'y', 'z'), translation)),
            'rotation': dict(zip(('x', 'y', 'z', 'w'), rotation)),
            'marker': {
                'dictionary': 'DICT_5X5_250',
                'id': 1,
                'size_m': 0.070,
            },
        },
    }


def export_calibration(input_path, output_path):
    if os.fspath(input_path) in ('latest', 'latest_real'):
        source = latest_calibration_path(simulation=False, kind='calib')
    elif os.fspath(input_path) == 'latest_sim':
        source = latest_calibration_path(simulation=True, kind='calib')
    else:
        source = expanded_path(input_path)
    with source.open(encoding='utf-8') as stream:
        normalized = normalized_calibration(yaml.safe_load(stream))
    return _write_normalized_calibration(normalized, output_path)


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
    # Validate before creating any temporary or destination file.  In
    # particular, a quality-gated caller cannot replace a known-good
    # deployment calibration with an invalid transform.
    normalized_calibration(source)
    return source


def write_calibration(transform, output_path):
    """Atomically write a WVCSC ``tool0 -> camera`` calibration transform."""
    source = _easy_handeye_calibration(transform)
    return _write_normalized_calibration(normalized_calibration(source), output_path)


def write_calibration_outputs(
        transform, deployment_output_path, normalized_output_path):
    """Atomically export matching easy_handeye2 and WVCSC calibration files.

    Both payloads are validated and fully staged before either destination is
    replaced.  Each destination replacement is atomic; a staging failure
    leaves both previously deployed calibrations untouched.
    """
    source = _easy_handeye_calibration(transform)
    normalized = normalized_calibration(source)
    deployment, normalized_output = _write_yaml_payloads_atomically([
        (source, deployment_output_path),
        (normalized, normalized_output_path),
    ])
    return deployment, normalized_output


def _write_normalized_calibration(normalized, output_path):
    return _write_yaml_payloads_atomically([(normalized, output_path)])[0]


def _stage_bytes(destination, payload):
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{destination.name}.', suffix='.tmp',
        dir=str(destination.parent))
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
    """Stage all YAML files before atomically replacing each destination."""
    staged = []
    backups = []
    installed = []
    destinations = []
    try:
        for document, output_path in payloads:
            destination = expanded_path(output_path)
            if destination in destinations:
                raise ValueError(f'duplicate calibration output path: {destination}')
            destinations.append(destination)
            payload = yaml.safe_dump(document, sort_keys=False).encode('utf-8')
            staged.append((destination, _stage_bytes(destination, payload)))

        # Preserve an existing value while applying the second file, so an
        # unexpected replace failure can restore the first destination.
        for destination, _temporary in staged:
            backup = (
                _stage_bytes(destination, destination.read_bytes())
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
        description='Validate and export easy_handeye2 C10 calibration.')
    parser.add_argument('--input', default='latest_real')
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    output_path = args.output
    if not output_path:
        _native, output_path = timestamped_calibration_paths()
    output = export_calibration(args.input, output_path)
    print(f'Validated WVCSC hand-eye calibration written to {output}')

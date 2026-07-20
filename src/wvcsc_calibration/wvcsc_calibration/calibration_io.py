"""Validate and export easy_handeye2 output to the WVCSC deployment file."""

import argparse
import math
import os
from pathlib import Path
import tempfile

import yaml


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
    source = Path(input_path).expanduser()
    destination = Path(output_path).expanduser()
    with source.open(encoding='utf-8') as stream:
        normalized = normalized_calibration(yaml.safe_load(stream))
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(normalized, sort_keys=False)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{destination.name}.', suffix='.tmp',
        dir=str(destination.parent))
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def main():
    parser = argparse.ArgumentParser(
        description='Validate and export easy_handeye2 C10 calibration.')
    parser.add_argument(
        '--input', default='~/.ros2/easy_handeye2/calibrations/wvcsc_c10.calib')
    parser.add_argument(
        '--output', default='~/.ros/wvcsc_calibration/c10_handeye.yaml')
    args = parser.parse_args()
    output = export_calibration(args.input, args.output)
    print(f'Validated WVCSC hand-eye calibration written to {output}')

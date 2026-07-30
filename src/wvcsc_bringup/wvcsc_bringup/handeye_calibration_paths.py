"""Shared C10 hand-eye result-file naming and selection rules."""

from datetime import datetime
import os
from pathlib import Path
import re


_TIMESTAMPED_CALIBRATION = re.compile(
    r'^c10_handeye(?P<sim>_sim)?_(?P<stamp>\d{8}_\d{6})\.(?P<kind>calib|samples)$')


def expanded_path(path):
    """Expand the portable ``$HOME`` and ``~`` forms used by calibration YAML."""
    return Path(os.path.expanduser(os.path.expandvars(os.fspath(path))))


def calibration_root_dir():
    """Return the shared source-workspace C10 calibration configuration root."""
    return (Path.home() / 'WVCSC_S2Z_UTB_ARM' / 'src' /
            'wvcsc_perception' / 'wvcsc_calibration' / 'config')


def calibration_config_dir(*, simulation=False):
    """Return the isolated real or Gazebo C10 calibration-result directory."""
    return calibration_root_dir() / ('sim' if simulation else 'real')


def timestamped_calibration_paths(output_dir=None, *, simulation=False,
                                  timestamp=None):
    """Return same-stamp native calibration and sample-archive paths."""
    directory = expanded_path(output_dir or calibration_config_dir(
        simulation=simulation))
    stamp = timestamp or datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')
    if not re.fullmatch(r'\d{8}_\d{6}', stamp):
        raise ValueError('calibration timestamp must be YYYYMMDD_HHMMSS')
    prefix = 'c10_handeye_sim' if simulation else 'c10_handeye'
    stem = f'{prefix}_{stamp}'
    return directory / f'{stem}.calib', directory / f'{stem}.samples'


def latest_calibration_path(directory=None, *, simulation=False, kind='calib'):
    """Select the newest timestamped native calibration for one environment."""
    if kind not in ('calib', 'samples'):
        raise ValueError("calibration kind must be 'calib' or 'samples'")
    directory = expanded_path(directory or calibration_config_dir(
        simulation=simulation))
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


def resolve_handeye_calibration(value, *, default_simulation=False):
    """Resolve a selector token or explicit C10 hand-eye calibration path."""
    value = os.fspath(value)
    if value in ('', 'latest'):
        return latest_calibration_path(simulation=default_simulation)
    if value == 'latest_real':
        return latest_calibration_path(simulation=False)
    if value == 'latest_sim':
        return latest_calibration_path(simulation=True)
    return expanded_path(value)

import os

import pytest

from wvcsc_bringup import handeye_calibration_paths
from wvcsc_bringup import path_defaults


def test_latest_timestamp_file_uses_timestamp_not_directory_order(tmp_path):
    older = tmp_path / 'map_20260724_090000'
    newer = tmp_path / 'map_20260724_100000'
    older.mkdir()
    newer.mkdir()
    (older / 'orchard.yaml').write_text('old', encoding='utf-8')
    (newer / 'orchard.yaml').write_text('new', encoding='utf-8')
    (tmp_path / 'map_invalid').mkdir()
    assert path_defaults._latest_timestamp_file(
        tmp_path, 'map', 'orchard.yaml') == newer / 'orchard.yaml'


def test_missing_timestamped_resources_fail_clearly(tmp_path, monkeypatch):
    monkeypatch.setattr(
        path_defaults, '_candidate_roots', lambda _package, _relative: (tmp_path,))
    with pytest.raises(RuntimeError, match='no timestamped map found'):
        path_defaults.latest_map_yaml()


def test_handeye_paths_keep_real_and_simulation_results_isolated(tmp_path):
    real, real_samples = handeye_calibration_paths.timestamped_calibration_paths(
        tmp_path / 'real', timestamp='20260730_100000')
    simulation, simulation_samples = (
        handeye_calibration_paths.timestamped_calibration_paths(
            tmp_path / 'sim', simulation=True, timestamp='20260730_100000'))

    assert real.name == 'c10_handeye_20260730_100000.calib'
    assert real_samples.name == 'c10_handeye_20260730_100000.samples'
    assert simulation.name == 'c10_handeye_sim_20260730_100000.calib'
    assert simulation_samples.name == 'c10_handeye_sim_20260730_100000.samples'


def test_latest_handeye_path_uses_embedded_timestamp_not_mtime(tmp_path):
    older = tmp_path / 'c10_handeye_20260730_090000.calib'
    newer = tmp_path / 'c10_handeye_20260730_100000.calib'
    simulation = tmp_path / 'c10_handeye_sim_20260730_110000.calib'
    for path in (older, newer, simulation):
        path.write_text(path.name, encoding='utf-8')
    os.utime(older, (2_000_000_000, 2_000_000_000))
    (tmp_path / 'c10_handeye_invalid.calib').write_text('', encoding='utf-8')

    assert handeye_calibration_paths.latest_calibration_path(tmp_path) == newer
    assert handeye_calibration_paths.latest_calibration_path(
        tmp_path, simulation=True) == simulation


def test_handeye_selector_tokens_have_fixed_environment_meaning(
        tmp_path, monkeypatch):
    monkeypatch.setattr(
        handeye_calibration_paths, 'calibration_root_dir', lambda: tmp_path)
    real_dir = tmp_path / 'real'
    simulation_dir = tmp_path / 'sim'
    real_dir.mkdir()
    simulation_dir.mkdir()
    real = real_dir / 'c10_handeye_20260730_100000.calib'
    simulation = simulation_dir / 'c10_handeye_sim_20260730_100000.calib'
    real.write_text('', encoding='utf-8')
    simulation.write_text('', encoding='utf-8')
    explicit = tmp_path / 'explicit.calib'
    explicit.write_text('', encoding='utf-8')
    monkeypatch.setenv('WVCSC_HAND_EYE_EXPLICIT', str(explicit))

    assert handeye_calibration_paths.resolve_handeye_calibration(
        '', default_simulation=True) == simulation
    assert handeye_calibration_paths.resolve_handeye_calibration(
        'latest', default_simulation=False) == real
    assert handeye_calibration_paths.resolve_handeye_calibration(
        'latest_real', default_simulation=True) == real
    assert handeye_calibration_paths.resolve_handeye_calibration(
        'latest_sim') == simulation
    assert handeye_calibration_paths.resolve_handeye_calibration(
        '$WVCSC_HAND_EYE_EXPLICIT') == explicit


def test_missing_or_invalid_handeye_result_fails_clearly(tmp_path):
    with pytest.raises(FileNotFoundError, match='simulation'):
        handeye_calibration_paths.latest_calibration_path(
            tmp_path, simulation=True)
    with pytest.raises(ValueError, match='calib'):
        handeye_calibration_paths.latest_calibration_path(
            tmp_path, kind='yaml')

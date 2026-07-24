from pathlib import Path

import pytest
import yaml

from wvcsc_calibration import calibration_io
from wvcsc_calibration.calibration_io import (
    latest_calibration_path,
    expanded_path,
    normalized_calibration,
    timestamped_calibration_paths,
    write_calibration_outputs,
)


def _valid():
    return {
        'parameters': {
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'alicia_base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_color_optical_frame',
        },
        'transform': {
            'translation': {'x': -0.055, 'y': 0.0, 'z': -0.10},
            'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        },
    }


def test_normalizes_valid_eye_in_hand_calibration():
    result = normalized_calibration(_valid())['calibration']
    assert result['parent_frame'] == 'tool0'
    assert result['marker']['size_m'] == pytest.approx(0.070)


def test_rejects_wrong_robot_contract():
    data = _valid()
    data['parameters']['robot_effector_frame'] = 'link6'
    with pytest.raises(ValueError, match='tool0'):
        normalized_calibration(data)


def test_rejects_non_unit_quaternion():
    data = _valid()
    data['transform']['rotation']['w'] = 0.1
    with pytest.raises(ValueError, match='quaternion norm'):
        normalized_calibration(data)


def test_expanded_path_supports_portable_home_environment_variable(monkeypatch, tmp_path):
    monkeypatch.setenv('WVCSC_CALIBRATION_TEST_ROOT', str(tmp_path))
    assert expanded_path('$WVCSC_CALIBRATION_TEST_ROOT/result.yaml') == (
        tmp_path / 'result.yaml')


def test_latest_calibration_selects_real_and_simulation_independently(tmp_path):
    (tmp_path / 'c10_handeye_20260724_090000.calib').write_text('real-old')
    (tmp_path / 'c10_handeye_20260724_100000.calib').write_text('real-new')
    (tmp_path / 'c10_handeye_sim_20260724_110000.calib').write_text('sim')
    assert latest_calibration_path(tmp_path).name == \
        'c10_handeye_20260724_100000.calib'
    assert latest_calibration_path(tmp_path, simulation=True).name == \
        'c10_handeye_sim_20260724_110000.calib'


def test_timestamped_paths_share_one_stem(tmp_path):
    native, normalized = timestamped_calibration_paths(
        tmp_path, timestamp='20260724_120000')
    assert native.name == 'c10_handeye_20260724_120000.calib'
    assert normalized.name == 'c10_handeye_20260724_120000.yaml'


def test_exported_easy_handeye_and_deployment_files_describe_same_transform(tmp_path):
    transform = ((0.01, -0.02, -0.10), (0.0, 0.0, 0.0, 1.0))
    deployment, normalized = write_calibration_outputs(
        transform,
        tmp_path / 'wvcsc_c10.calib',
        tmp_path / 'c10_handeye.yaml')

    easy_handeye = yaml.safe_load(deployment.read_text(encoding='utf-8'))
    deployment_data = yaml.safe_load(normalized.read_text(encoding='utf-8'))
    assert easy_handeye['parameters'] == {
        'name': 'wvcsc_c10',
        'calibration_type': 'eye_in_hand',
        'robot_base_frame': 'alicia_base_link',
        'robot_effector_frame': 'tool0',
        'tracking_base_frame': 'camera_color_optical_frame',
        'tracking_marker_frame': 'calibration_aruco',
        'freehand_robot_movement': True,
        'move_group_namespace': '/',
        'move_group': 'arm',
    }
    assert easy_handeye['transform']['translation'] == \
        deployment_data['calibration']['translation']
    assert easy_handeye['transform']['rotation'] == \
        deployment_data['calibration']['rotation']


def test_invalid_or_unwritable_export_does_not_replace_existing_calibration(
        monkeypatch, tmp_path):
    deployment = tmp_path / 'wvcsc_c10.calib'
    normalized = tmp_path / 'c10_handeye.yaml'
    deployment.write_text('old_easy_handeye\n', encoding='utf-8')
    normalized.write_text('old_deployment\n', encoding='utf-8')

    with pytest.raises(ValueError, match='camera translation'):
        write_calibration_outputs(
            ((0.60, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            deployment, normalized)
    assert deployment.read_text(encoding='utf-8') == 'old_easy_handeye\n'
    assert normalized.read_text(encoding='utf-8') == 'old_deployment\n'

    real_stage = calibration_io._stage_bytes
    calls = {'count': 0}

    def fail_while_staging(destination, payload):
        calls['count'] += 1
        if calls['count'] == 2:
            raise OSError('simulated staging failure')
        return real_stage(destination, payload)

    monkeypatch.setattr(calibration_io, '_stage_bytes', fail_while_staging)
    with pytest.raises(OSError, match='staging failure'):
        write_calibration_outputs(
            ((0.01, 0.0, -0.10), (0.0, 0.0, 0.0, 1.0)),
            deployment, normalized)
    assert deployment.read_text(encoding='utf-8') == 'old_easy_handeye\n'
    assert normalized.read_text(encoding='utf-8') == 'old_deployment\n'

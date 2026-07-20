import pytest

from wvcsc_calibration.calibration_io import normalized_calibration


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

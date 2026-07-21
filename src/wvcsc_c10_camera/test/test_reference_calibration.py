from pathlib import Path

import yaml


def test_reference_calibration_matches_gazebo_c10_model():
    path = (
        Path(__file__).resolve().parents[1]
        / 'config' / 'c10_intrinsics.yaml'
    )
    calibration = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert calibration['image_width'] == 1280
    assert calibration['image_height'] == 720
    assert calibration['distortion_coefficients']['data'] == [0.0] * 5
    assert calibration['camera_matrix']['data'] == [
        507.872735, 0.0, 640.5,
        0.0, 507.872735, 360.5,
        0.0, 0.0, 1.0,
    ]

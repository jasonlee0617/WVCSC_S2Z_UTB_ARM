import math
from pathlib import Path

import pytest
import yaml


def test_reference_calibration_is_the_gazebo_c10_source_of_truth():
    package = Path(__file__).resolve().parents[1]
    calibration = yaml.safe_load((package / 'config' / 'c10_intrinsics.yaml').read_text(
        encoding='utf-8'))
    assert calibration['image_width'] == 640
    assert calibration['image_height'] == 480
    assert calibration['camera_matrix']['data'] == [
        539.555860, 0.0, 328.213731,
        0.0, 541.478543, 262.872433,
        0.0, 0.0, 1.0,
    ]
    assert calibration['distortion_coefficients']['data'] == [
        -0.032625, -0.023878, 0.003795, 0.004174, 0.0,
    ]
    assert 2.0 * math.atan(640.0 / (2.0 * 539.555860)) == pytest.approx(
        1.0706320326518812)

    xacro = (package.parents[0] / 'wvcsc_description' / 'urdf' /
             'wvcsc_utb_alicia.urdf.xacro').read_text(encoding='utf-8')
    assert "xacro.load_yaml('$(find wvcsc_c10_camera)/config/c10_intrinsics.yaml')" in xacro
    assert '<intrinsics>' in xacro
    assert '<distortion>' in xacro
    assert '<P_fy>${c10_fy}</P_fy>' in xacro
    assert '<border_crop>false</border_crop>' in xacro

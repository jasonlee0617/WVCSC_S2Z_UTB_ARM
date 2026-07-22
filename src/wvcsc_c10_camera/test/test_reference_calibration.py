import math
from pathlib import Path

import pytest
import yaml


def test_reference_calibration_is_the_gazebo_c10_source_of_truth():
    package = Path(__file__).resolve().parents[1]
    calibration = yaml.safe_load((package / 'config' / 'c10_intrinsics.yaml').read_text(
        encoding='utf-8'))
    assert calibration['image_width'] == 1280
    assert calibration['image_height'] == 720
    assert calibration['camera_matrix']['data'] == [
        1079.11172, 0.0, 656.42746,
        0.0, 1082.95708, 525.74486,
        0.0, 0.0, 1.0,
    ]
    assert calibration['distortion_coefficients']['data'] == [
        -0.032625, -0.023878, 0.003795, 0.004174, 0.0,
    ]
    assert 2.0 * math.atan(1280.0 / (2.0 * 1079.11172)) == pytest.approx(
        1.0706320326518812)

    xacro = (package.parents[0] / 'wvcsc_description' / 'urdf' /
             'wvcsc_utb_alicia.urdf.xacro').read_text(encoding='utf-8')
    assert "xacro.load_yaml('$(find wvcsc_c10_camera)/config/c10_intrinsics.yaml')" in xacro
    assert '<intrinsics>' in xacro
    assert '<distortion>' in xacro
    assert '<P_fy>${c10_fy}</P_fy>' in xacro
    assert '<border_crop>false</border_crop>' in xacro

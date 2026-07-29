import math
from pathlib import Path

import pytest

from wvcsc_simulation.ackermann_math import (
    point_to_polyline_distance,
    point_to_segment_distance,
    yaw_rate_from_steering,
    yaw_rate_from_twist,
)


def test_node_defaults_match_confirmed_real_vehicle_geometry():
    source = (Path(__file__).parents[1] / 'scripts' /
              'ackermann_sim.py').read_text(encoding='utf-8')

    assert "declare_parameter('wheel_base', 0.70)" in source
    assert "declare_parameter('cmd_angular_mode', 'yaw_rate')" in source


def test_confirmed_real_geometry_has_expected_minimum_turning_radius():
    wheel_base = 0.70
    max_steering_angle = 0.48

    assert wheel_base / math.tan(max_steering_angle) == pytest.approx(1.3445744)


def test_steering_angle_command_uses_ackermann_kinematics():
    assert yaw_rate_from_steering(0.35, 0.20, 0.70) == pytest.approx(
        0.35 * math.tan(0.20) / 0.70)


def test_reverse_speed_preserves_ackermann_yaw_sign():
    assert yaw_rate_from_steering(-0.20, 0.20, 0.82) < 0.0


def test_standard_twist_yaw_rate_is_limited_by_ackermann_curvature():
    expected = 0.35 * math.tan(0.48) / 0.70

    assert yaw_rate_from_twist(0.35, 1.0, 0.70, 0.48) == pytest.approx(expected)
    assert yaw_rate_from_twist(0.35, -1.0, 0.70, 0.48) == pytest.approx(-expected)
    assert yaw_rate_from_twist(-0.35, 0.10, 0.70, 0.48) == pytest.approx(0.10)


def test_cross_track_distance_uses_the_finite_active_route_segment():
    assert point_to_segment_distance(0.5, 0.1, (0.0, 0.0), (1.0, 0.0)) == \
        pytest.approx(0.1)
    assert point_to_segment_distance(2.0, 0.0, (0.0, 0.0), (1.0, 0.0)) == \
        pytest.approx(1.0)


def test_controller_path_error_uses_the_closest_polyline_segment():
    assert point_to_polyline_distance(
        1.0, 0.2, ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))) == pytest.approx(0.0)
    assert math.isnan(point_to_polyline_distance(1.2, 0.5, ()))

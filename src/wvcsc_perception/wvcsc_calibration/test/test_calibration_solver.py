import math
from types import SimpleNamespace

import numpy as np
import pytest

from wvcsc_calibration.calibration_quality import transform_error
from wvcsc_calibration.calibration_solver import (
    invert_transform,
    matrix_quaternion,
    quaternion_matrix,
    refine_handeye_fixed_marker,
    solve_handeye,
)


def _transform(components):
    translation, rotation = components
    return SimpleNamespace(
        translation=SimpleNamespace(x=translation[0], y=translation[1], z=translation[2]),
        rotation=SimpleNamespace(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]))


def _compose(left, right):
    left_translation, left_quaternion = left
    right_translation, right_quaternion = right
    left_rotation = quaternion_matrix(left_quaternion)
    right_rotation = quaternion_matrix(right_quaternion)
    return (
        tuple(left_rotation @ np.asarray(right_translation) + left_translation),
        matrix_quaternion(left_rotation @ right_rotation),
    )


def _rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                     (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                     (-sp, cp * sr, cp * cr)))


def test_solver_preserves_ros_forward_sample_convention():
    expected = ((-0.055, 0.0, -0.10), (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)))
    base_marker = ((0.45, 0.0, 0.752), (0.0, 0.0, 0.0, 1.0))
    samples = []
    for index in range(12):
        base_tool = (
            (0.10 + 0.015 * index, -0.08 + 0.017 * (index % 5), 0.82 + 0.01 * (index % 3)),
            matrix_quaternion(_rpy(
                math.radians(-24 + 5 * index),
                math.radians(-18 + 3 * (index % 6)),
                math.radians(-30 + 7 * (index % 7)))),
        )
        camera_marker = _compose(
            _compose(invert_transform(expected), invert_transform(base_tool)),
            base_marker)
        samples.append(SimpleNamespace(
            robot=_transform(base_tool), tracking=_transform(camera_marker)))

    results = solve_handeye(
        samples, ('OpenCV/Park', 'OpenCV/Horaud', 'OpenCV/Tsai-Lenz'))
    for transform in results.values():
        translation, rotation = transform_error(transform, expected)
        assert translation < 1.0e-6
        assert rotation < 1.0e-4


def test_fixed_marker_refinement_preserves_an_exact_handeye_solution():
    expected = ((-0.055, 0.0, -0.10), (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5)))
    base_marker = ((0.0, 0.25, 0.003), (0.0, 0.0, 1.0, 0.0))
    samples = []
    for index in range(8):
        base_tool = (
            (0.08 + 0.014 * index, -0.06 + 0.012 * (index % 4),
             0.78 + 0.018 * (index % 3)),
            matrix_quaternion(_rpy(
                math.radians(-20 + 6 * index),
                math.radians(-15 + 5 * (index % 5)),
                math.radians(-25 + 9 * (index % 6)))),
        )
        camera_marker = _compose(
            _compose(invert_transform(expected), invert_transform(base_tool)),
            base_marker)
        samples.append(SimpleNamespace(
            robot=_transform(base_tool), tracking=_transform(camera_marker)))

    refined, details = refine_handeye_fixed_marker(
        samples, expected, translation_sigma_m=0.00025,
        rotation_sigma_deg=1.0, max_iterations=25)
    translation, rotation = transform_error(refined, expected)
    assert translation < 1.0e-8
    assert rotation < 1.0e-5
    assert details['final_cost'] <= details['initial_cost']


def test_invert_transform_round_trip_preserves_rigid_pose():
    transform = ((0.12, -0.04, 0.08), matrix_quaternion(_rpy(0.3, -0.2, 0.5)))
    identity = _compose(transform, invert_transform(transform))
    assert identity[0] == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-9)
    assert identity[1] == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1.0e-9)

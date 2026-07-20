import math

import pytest

from wvcsc_calibration.alicia_sample_geometry import generate_alicia_candidates
from wvcsc_arm_task.observation import rotate_vector


def test_alicia_candidates_are_marker_relative_and_complete():
    starting_quaternion = (
        0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
    candidates = generate_alicia_candidates(
        marker_position=(0.8, 0.0, 0.7),
        current_camera_position=(0.3, 0.0, 0.7),
        current_camera_quaternion=starting_quaternion,
        tool_to_camera_translation=(-0.055, 0.0, -0.10),
        tool_to_camera_quaternion=(0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)),
    )

    assert len(candidates) == 21
    assert len({candidate.candidate_id for candidate in candidates}) == 21
    assert candidates[0].candidate_id == 'seed'
    assert candidates[0].camera_position == pytest.approx((0.3, 0.0, 0.7))
    assert candidates[0].camera_quaternion == pytest.approx(starting_quaternion)
    for candidate in candidates:
        optical_z = rotate_vector((0.0, 0.0, 1.0), candidate.camera_quaternion)
        marker_ray = tuple(
            (0.8, 0.0, 0.7)[index] - candidate.camera_position[index]
            for index in range(3))
        marker_norm = math.sqrt(sum(value * value for value in marker_ray))
        marker_ray = tuple(value / marker_norm for value in marker_ray)
        assert sum(a * b for a, b in zip(optical_z, marker_ray)) > 0.999


def test_alicia_candidates_reject_unsafe_initial_range():
    with pytest.raises(ValueError, match='distance is unsafe'):
        generate_alicia_candidates(
            (0.0, 0.0, 0.0), (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

import math

import pytest

from wvcsc_calibration.alicia_sample_geometry import (
    generate_alicia_candidates,
    generate_initial_anchor_candidates,
)
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


def test_initial_anchors_follow_marker_prior_and_aim_c10_at_marker():
    marker = (0.0, 0.25, 0.002)
    candidates = generate_initial_anchor_candidates(
        marker_position=marker,
        tool_to_camera_translation=(-0.055, 0.0, -0.10),
        tool_to_camera_quaternion=(
            0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)),
        height_candidates=(0.25, 0.30, 0.35),
        radial_backoff_candidates=(0.0, 0.05, 0.10),
        tangential_offset_candidates=(0.0, -0.05, 0.05),
    )
    assert len(candidates) == 27
    assert len({candidate.candidate_id for candidate in candidates}) == 27
    for candidate in candidates:
        optical_z = rotate_vector((0.0, 0.0, 1.0), candidate.camera_quaternion)
        ray = tuple(marker[index] - candidate.camera_position[index]
                    for index in range(3))
        norm = math.sqrt(sum(value * value for value in ray))
        assert sum(left * right for left, right in zip(
            optical_z, (value / norm for value in ray))) > 0.999


def test_initial_anchors_reject_a_marker_at_the_arm_base():
    with pytest.raises(ValueError, match='offset from the arm base'):
        generate_initial_anchor_candidates(
            marker_position=(0.0, 0.0, 0.0),
            tool_to_camera_translation=(0.0, 0.0, 0.0),
            tool_to_camera_quaternion=(0.0, 0.0, 0.0, 1.0),
            height_candidates=(0.40,),
            radial_backoff_candidates=(0.10,),
            tangential_offset_candidates=(0.0,))


def test_image_centre_aim_offsets_project_marker_away_from_offset_principal_point():
    marker = (0.0, 0.25, 0.002)
    fx, fy = 1079.11172, 1082.95708
    cx, cy = 656.42746, 525.74486
    width, height = 1280.0, 720.0
    pitch = math.atan2(height * 0.5 - cy, fy)
    yaw = math.atan2((cx - width * 0.5) * math.cos(pitch), fx)
    candidate = generate_initial_anchor_candidates(
        marker_position=marker,
        tool_to_camera_translation=(-0.055, 0.0, -0.10),
        tool_to_camera_quaternion=(
            0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)),
        height_candidates=(0.25,),
        radial_backoff_candidates=(0.0,),
        tangential_offset_candidates=(0.0,),
        aim_yaw_deg=math.degrees(yaw),
        aim_pitch_deg=math.degrees(pitch),
    )[0]
    ray = tuple(marker[index] - candidate.camera_position[index]
                for index in range(3))
    quaternion = candidate.camera_quaternion
    camera_ray = rotate_vector(
        ray, (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]))
    projected = (
        fx * camera_ray[0] / camera_ray[2] + cx,
        fy * camera_ray[1] / camera_ray[2] + cy,
    )
    assert projected == pytest.approx((width * 0.5, height * 0.5), abs=1.0e-6)


def test_alicia_fine_expansion_preserves_marker_aiming_and_unique_ids():
    marker = (0.8, 0.0, 0.7)
    candidates = generate_alicia_candidates(
        marker_position=marker,
        current_camera_position=(0.3, 0.0, 0.7),
        current_camera_quaternion=(
            0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)),
        tool_to_camera_translation=(-0.055, 0.0, -0.10),
        tool_to_camera_quaternion=(
            0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)),
        include_fine=True)
    assert len(candidates) == 49
    assert len({candidate.candidate_id for candidate in candidates}) == 49
    assert any(candidate.candidate_id.startswith('wide_')
               for candidate in candidates)
    assert any(candidate.candidate_id.startswith('fine_')
               for candidate in candidates)
    for candidate in candidates:
        optical_z = rotate_vector((0.0, 0.0, 1.0), candidate.camera_quaternion)
        marker_ray = tuple(
            marker[index] - candidate.camera_position[index]
            for index in range(3))
        norm = math.sqrt(sum(value * value for value in marker_ray))
        alignment = sum(left * right for left, right in zip(
            optical_z, (value / norm for value in marker_ray)))
        if candidate.candidate_id.startswith('wide_tilt_'):
            assert alignment > 0.94
        else:
            assert alignment > 0.999

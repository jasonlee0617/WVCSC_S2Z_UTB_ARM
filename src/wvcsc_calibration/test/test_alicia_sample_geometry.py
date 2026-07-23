import math

import pytest

from wvcsc_calibration.alicia_sample_geometry import (
    generate_alicia_candidates,
    generate_initial_anchor_candidates,
)
from wvcsc_arm_task.observation import rotate_vector


def _marker_alignment(candidate, marker):
    tool_z = rotate_vector((0.0, 0.0, 1.0), candidate.tool_quaternion)
    ray = tuple(marker[index] - candidate.tool_position[index]
                for index in range(3))
    norm = math.sqrt(sum(value * value for value in ray))
    return sum(left * right for left, right in zip(
        tool_z, (value / norm for value in ray)))


def test_alicia_candidates_are_marker_relative_and_complete():
    marker = (0.8, 0.0, 0.7)
    candidates = generate_alicia_candidates(
        marker_position=marker,
        current_tool_position=(0.3, 0.0, 0.7),
        current_tool_quaternion=(
            0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)),
    )

    assert len(candidates) == 21
    assert len({candidate.candidate_id for candidate in candidates}) == 21
    assert candidates[0].candidate_id == 'seed'
    assert candidates[0].tool_position == pytest.approx((0.3, 0.0, 0.7))
    for candidate in candidates[1:]:
        assert _marker_alignment(candidate, marker) > 0.999


def test_alicia_candidates_reject_unsafe_initial_range():
    with pytest.raises(ValueError, match='distance is unsafe'):
        generate_alicia_candidates(
            (0.0, 0.0, 0.0), (2.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))


def test_initial_anchor_primary_pose_is_directly_above_marker_and_z_down():
    marker = (0.0, 0.25, 0.002)
    candidates = generate_initial_anchor_candidates(
        marker_position=marker,
        height_candidates=(0.30, 0.40),
        radial_backoff_candidates=(0.0, 0.05),
        tangential_offset_candidates=(0.0, -0.05, 0.05),
    )
    # 12 direct/backoff/tangential anchors plus two tool-space visibility
    # probes for each of the two direct heights.
    assert len(candidates) == 16
    center = next(candidate for candidate in candidates
                  if candidate.candidate_id == 'initial_h0.300_r0.000_t+0.000')
    assert center.tool_position == pytest.approx((0.0, 0.25, 0.302))
    assert rotate_vector((0.0, 0.0, 1.0), center.tool_quaternion) == pytest.approx(
        (0.0, 0.0, -1.0))
    assert _marker_alignment(center, marker) == pytest.approx(1.0)
    tilted = [candidate for candidate in candidates
              if candidate.candidate_id.startswith('initial_h0.300_r0.000_t+0.000_yaw')]
    assert len(tilted) == 2
    assert all(_marker_alignment(candidate, marker) > 0.9
               for candidate in tilted)
    fallback = next(candidate for candidate in candidates
                    if candidate.candidate_id == 'initial_h0.300_r0.050_t+0.000')
    assert _marker_alignment(fallback, marker) > 0.999


def test_initial_anchors_reject_a_marker_at_the_arm_base():
    with pytest.raises(ValueError, match='offset from the arm base'):
        generate_initial_anchor_candidates(
            marker_position=(0.0, 0.0, 0.0),
            height_candidates=(0.40,),
            radial_backoff_candidates=(0.10,),
            tangential_offset_candidates=(0.0,))


def test_fine_expansion_has_wide_and_fine_rotation_excitation():
    marker = (0.8, 0.0, 0.7)
    candidates = generate_alicia_candidates(
        marker_position=marker,
        current_tool_position=(0.3, 0.0, 0.7),
        current_tool_quaternion=(
            0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)),
        include_fine=True)
    assert len(candidates) == 49
    assert len({candidate.candidate_id for candidate in candidates}) == 49
    assert any(candidate.candidate_id.startswith('wide_')
               for candidate in candidates)
    assert any(candidate.candidate_id.startswith('fine_')
               for candidate in candidates)
    for candidate in candidates:
        assert _marker_alignment(candidate, marker) > 0.94


def test_candidate_generation_does_not_depend_on_camera_mount():
    marker = (0.8, 0.0, 0.7)
    first = generate_alicia_candidates(
        marker, (0.3, 0.0, 0.7), (0.0, 0.0, 0.0, 1.0), include_fine=True)
    second = generate_alicia_candidates(
        marker, (0.3, 0.0, 0.7), (0.0, 0.0, 0.0, 1.0), include_fine=True)
    assert [item.candidate_id for item in first] == [
        item.candidate_id for item in second]
    for left, right in zip(first, second):
        assert left.tool_position == pytest.approx(right.tool_position)
        assert left.tool_quaternion == pytest.approx(right.tool_quaternion)

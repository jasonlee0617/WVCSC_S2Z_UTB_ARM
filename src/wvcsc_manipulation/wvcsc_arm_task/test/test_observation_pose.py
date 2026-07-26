import math

import pytest

from wvcsc_arm_task.ik_observation import (
    camera_look_at_pose,
    recenter_camera_pose,
    rotation_matrix_from_quaternion,
    rotate_vector,
    tool_pose_from_camera_pose,
    transform_point,
)


def _forward_from_quaternion(quat):
    x, y, z, w = quat
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


@pytest.mark.parametrize(
    ('tree_y', 'camera_y', 'forward_y'),
    ((1.5, 0.4, 1.0), (-1.5, -0.4, -1.0)),
)
def test_camera_look_at_pose_keeps_distance_and_aims_optical_z_at_tree(
        tree_y, camera_y, forward_y):
    position, quat = camera_look_at_pose(
        (0.0, tree_y, -1.32), 1.20, 0.30, 1.10)
    assert position == pytest.approx((0.0, camera_y, 0.30))
    direction = (0.0, forward_y * 1.10, -0.42)
    norm = math.sqrt(sum(value * value for value in direction))
    assert _forward_from_quaternion(quat) == pytest.approx(
        tuple(value / norm for value in direction))


def test_camera_look_at_pose_rejects_tree_inside_observation_clearance():
    with pytest.raises(ValueError, match='too close'):
        camera_look_at_pose((0.0, 1.10, 0.0), 1.20, 0.30, 1.10)


def test_transform_point_applies_translation_and_rotation():
    point = transform_point(
        (1.0, 0.0, 0.0), (0.0, 2.0, 0.0),
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
    )
    assert point == pytest.approx((0.0, 3.0, 0.0))


def test_tool_pose_from_camera_pose_inverts_fixed_camera_translation():
    position, quat = tool_pose_from_camera_pose(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0),
        (0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
    )
    assert position == pytest.approx((0.9, 2.0, 3.0))
    assert quat == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_rotation_matrix_and_rotate_vector_share_the_same_geometry():
    quaternion = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    matrix = rotation_matrix_from_quaternion(quaternion)
    assert tuple(row[0] for row in matrix) == pytest.approx((0.0, 1.0, 0.0))
    assert rotate_vector((1.0, 0.0, 0.0), quaternion) == pytest.approx(
        (0.0, 1.0, 0.0))


def test_recenter_keeps_camera_position_and_maps_target_to_desired_spray_ray():
    camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    position, quat, angle = recenter_camera_pose(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), camera,
        740.0, 360.0, 640.0, 388.0, 18.0)
    source_ray = (0.2, 0.0, 1.0)
    desired_ray = (0.0, 28.0 / 500.0, 1.0)
    source_norm = math.sqrt(sum(value * value for value in source_ray))
    desired_norm = math.sqrt(sum(value * value for value in desired_ray))
    assert position == pytest.approx((1.0, 2.0, 3.0))
    assert angle < 18.0
    assert rotate_vector(
        tuple(value / desired_norm for value in desired_ray), quat) == pytest.approx(
            tuple(value / source_norm for value in source_ray))


def test_partial_recenter_leaves_a_configured_pixel_residual_for_ibvs():
    camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    position, quat, angle = recenter_camera_pose(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), camera,
        740.0, 360.0, 640.0, 388.0, 18.0, residual_error_px=40.0)
    source_ray = (0.2, 0.0, 1.0)
    # Original error is (+100, -28) px. Scaling its largest axis to 40 px
    # leaves the intermediate aim at (680, 376.8).
    partial_ray = (40.0 / 500.0, 16.8 / 500.0, 1.0)
    source_norm = math.sqrt(sum(value * value for value in source_ray))
    partial_norm = math.sqrt(sum(value * value for value in partial_ray))

    assert position == pytest.approx((1.0, 2.0, 3.0))
    assert angle < 18.0
    rotated = rotate_vector(
        tuple(value / partial_norm for value in partial_ray), quat)
    assert rotated == pytest.approx(
        tuple(value / source_norm for value in source_ray))


def test_partial_recenter_can_fit_when_exact_recentering_exceeds_angle_limit():
    camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    with pytest.raises(ValueError, match='angle exceeds limit'):
        recenter_camera_pose(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), camera,
            820.0, 388.0, 640.0, 388.0, 18.0)

    _position, _quat, angle = recenter_camera_pose(
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), camera,
        820.0, 388.0, 640.0, 388.0, 18.0, residual_error_px=40.0)

    assert angle < 18.0


def test_recenter_rejects_a_rotation_larger_than_the_safety_limit():
    with pytest.raises(ValueError, match='angle exceeds limit'):
        recenter_camera_pose(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            (500.0, 500.0, 640.0, 360.0, 1280, 720),
            900.0, 360.0, 640.0, 388.0, 18.0)


def test_full_recenter_between_twenty_and_forty_five_degrees_is_allowed():
    camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    with pytest.raises(
            ValueError, match=r'required_angle=.*limit=20.0deg'):
        recenter_camera_pose(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), camera,
            900.0, 360.0, 640.0, 388.0, 20.0)

    _position, _quat, angle = recenter_camera_pose(
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), camera,
        900.0, 360.0, 640.0, 388.0, 45.0)

    assert 20.0 < angle < 45.0

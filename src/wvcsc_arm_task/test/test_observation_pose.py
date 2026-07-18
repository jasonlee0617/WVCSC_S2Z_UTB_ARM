import math

import pytest

from wvcsc_arm_task.observation_pose import (
    camera_look_at_pose,
    recenter_camera_pose,
    rotate_vector,
    tool_pose_from_camera_pose,
    transform_point,
    yaw_rotate_quaternion,
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
        (0.0, tree_y, -1.32), 1.20, 1.90, 1.10)
    assert position == pytest.approx((0.0, camera_y, 0.58))
    direction = (0.0, forward_y * 1.10, -0.70)
    norm = math.sqrt(sum(value * value for value in direction))
    assert _forward_from_quaternion(quat) == pytest.approx(
        tuple(value / norm for value in direction))


def test_camera_look_at_pose_rejects_tree_inside_observation_clearance():
    with pytest.raises(ValueError, match='too close'):
        camera_look_at_pose((0.0, 1.10, 0.0), 1.20, 1.90, 1.10)


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


def test_yaw_rotate_quaternion_keeps_a_horizontal_fan_at_fixed_camera_position():
    forward = _forward_from_quaternion(
        yaw_rotate_quaternion((0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)), 90.0))
    assert forward == pytest.approx((0.0, 1.0, 0.0))


def test_recenter_keeps_camera_position_and_maps_target_to_desired_spray_ray():
    camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    position, quat, angle = recenter_camera_pose(
        (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), camera,
        740.0, 360.0, 0.0, 28.0, 18.0)
    source_ray = (0.2, 0.0, 1.0)
    desired_ray = (0.0, 28.0 / 500.0, 1.0)
    source_norm = math.sqrt(sum(value * value for value in source_ray))
    desired_norm = math.sqrt(sum(value * value for value in desired_ray))
    assert position == pytest.approx((1.0, 2.0, 3.0))
    assert angle < 18.0
    assert rotate_vector(
        tuple(value / desired_norm for value in desired_ray), quat) == pytest.approx(
            tuple(value / source_norm for value in source_ray))


def test_recenter_rejects_a_rotation_larger_than_the_safety_limit():
    with pytest.raises(ValueError, match='angle exceeds limit'):
        recenter_camera_pose(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            (500.0, 500.0, 640.0, 360.0, 1280, 720),
            900.0, 360.0, 0.0, 28.0, 18.0)

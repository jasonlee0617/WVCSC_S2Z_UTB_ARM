import math

import pytest

from wvcsc_arm_task.observation import ObservationOptimizer


ROBOT = '''
<robot name="test">
  <link name="base"/><link name="link1"/><link name="link2"/>
  <link name="link3"/><link name="link4"/><link name="link5"/><link name="tip"/>
  <joint name="joint1" type="revolute"><parent link="base"/><child link="link1"/><origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="joint2" type="revolute"><parent link="link1"/><child link="link2"/><origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="joint3" type="revolute"><parent link="link2"/><child link="link3"/><origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="joint4" type="revolute"><parent link="link3"/><child link="link4"/><origin xyz="0.2 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="joint5" type="revolute"><parent link="link4"/><child link="link5"/><origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="joint6" type="revolute"><parent link="link5"/><child link="tip"/><origin xyz="0.1 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
</robot>
'''


def _optimizer(**overrides):
    config = {
        'fruit_zone_height_min_m': 0.7,
        'fruit_zone_height_max_m': 1.7,
        'fruit_zone_radius_m': 0.5,
        'distance_min_m': 1.3,
        'distance_max_m': 1.5,
        'distance_step_m': 0.1,
        'camera_height_min_m': 0.2,
        'camera_height_max_m': 0.4,
        'camera_height_step_m': 0.1,
        'azimuth_offsets_deg': (0.0,),
        'image_margin_ratio': 0.08,
        'min_visible_fraction': 0.60,
        'max_condition_number': 12.0,
        'min_joint_margin_rad': 0.15,
        'preferred_joint_margin_rad': 0.35,
    }
    config.update(overrides)
    return ObservationOptimizer(
        ROBOT, 'base', 'tip',
        ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'), config)


def _candidates(optimizer):
    return optimizer.generate(
        (2.0, 0.0, 0.0), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (600.0, 600.0, 640.0, 360.0, 1280, 720))


def test_candidates_keep_the_fruit_center_and_required_coverage():
    candidates = _candidates(_optimizer())

    assert candidates
    visible = [candidate for candidate in candidates if candidate.visible]
    assert visible
    assert all(isinstance(candidate.visible, bool) for candidate in candidates)
    assert all(candidate.visible_fraction >= 0.60 for candidate in visible)


def test_off_center_c10_intrinsics_keep_runtime_observation_candidates():
    optimizer = _optimizer(
        fruit_zone_height_min_m=0.8,
        fruit_zone_height_max_m=1.6,
        distance_min_m=1.1,
        distance_max_m=1.5,
        camera_height_min_m=0.2,
        camera_height_max_m=0.4,
        azimuth_offsets_deg=(0.0, -12.0, 12.0),
        image_margin_ratio=0.07,
    )
    candidates = optimizer.generate(
        (-0.21, -1.79, -1.55),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (1079.11172, 1082.95708, 656.42746, 525.74486, 1280, 720),
    )

    visible = [candidate for candidate in candidates if candidate.visible]
    assert visible
    assert max(candidate.visible_fraction for candidate in candidates) >= 0.60
    assert all(0.20 <= candidate.camera_position[2] <= 0.40
               for candidate in visible)
    assert all(candidate.target_u_px == pytest.approx(640.0) for candidate in visible)
    assert all(candidate.target_v_px == pytest.approx(360.0) for candidate in visible)


def test_camera_height_is_measured_from_the_arm_base():
    optimizer = _optimizer(
        camera_height_min_m=0.2,
        camera_height_max_m=0.4,
    )
    candidates = optimizer.generate(
        (2.0, 0.0, -1.55),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (600.0, 600.0, 640.0, 360.0, 1280, 720),
    )

    assert candidates
    assert sorted({candidate.camera_position[2] for candidate in candidates}) == \
        pytest.approx([0.2, 0.3, 0.4])


@pytest.mark.parametrize('tree_y', (-1.0, 1.0))
def test_observation_grid_requires_room_between_base_and_tree(tree_y):
    optimizer = _optimizer(
        distance_min_m=1.0,
        distance_max_m=1.0,
    )

    candidates = optimizer.generate(
        (0.0, tree_y, 0.0),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (600.0, 600.0, 640.0, 360.0, 1280, 720),
    )

    assert candidates == []


def test_tree_scan_orders_center_then_same_view_left_and_right_fan():
    optimizer = _optimizer(
        distance_min_m=1.3,
        distance_max_m=1.3,
        camera_height_min_m=0.2,
        camera_height_max_m=0.3,
        azimuth_offsets_deg=(0.0, -12.0, 12.0),
    )
    candidates = _candidates(optimizer)
    for candidate in candidates:
        candidate.visible = True
        candidate.rejection_reason = ''
        candidate.condition_number = 5.0
        candidate.min_joint_margin_rad = 0.5
        candidate.joint_motion_norm = 0.1

    ordered = optimizer.order_for_tree_scan(candidates)

    assert [candidate.azimuth_deg for candidate in ordered[:3]] == [0.0, -12.0, 12.0]
    assert [candidate.selection_phase for candidate in ordered[:3]] == [
        'center_initial', 'fan_left', 'fan_right']
    assert all(candidate.selection_phase == 'recovery' for candidate in ordered[3:])


def test_tree_scan_marks_lateral_fallback_when_no_center_is_safe():
    optimizer = _optimizer(azimuth_offsets_deg=(0.0, -12.0, 12.0))
    candidates = _candidates(optimizer)
    for candidate in candidates:
        candidate.visible = candidate.azimuth_deg != 0.0
        candidate.rejection_reason = ''
        candidate.condition_number = 5.0
        candidate.min_joint_margin_rad = 0.5
        candidate.joint_motion_norm = 0.1

    ordered = optimizer.order_for_tree_scan(candidates)

    assert ordered[0].selection_phase == 'center_unavailable_fallback'
    assert ordered[0].azimuth_deg == -12.0


def test_coverage_below_configured_threshold_is_rejected():
    candidates = _candidates(_optimizer(min_visible_fraction=1.0, fruit_zone_radius_m=2.0))

    assert candidates
    assert not any(candidate.visible for candidate in candidates)
    assert all(candidate.rejection_reason == 'fruit_zone_coverage_below_threshold'
               for candidate in candidates)


def test_joint_margin_rejects_a_near_limit_ik_solution():
    optimizer = _optimizer(max_condition_number=math.inf)
    candidate = _candidates(optimizer)[0]

    optimizer.evaluate_ik(candidate, (2.95, 0.0, -1.0, 0.4, 0.5, -0.7), (0.0,) * 6)

    assert candidate.rejection_reason == 'joint_limit_margin'
    assert candidate.min_joint_margin_rad == pytest.approx(0.05)


def test_singular_kinematics_are_never_ranked():
    optimizer = _optimizer()
    candidate = _candidates(optimizer)[0]

    optimizer.evaluate_ik(candidate, (0.0,) * 6, (0.0,) * 6)

    assert math.isinf(candidate.condition_number) or candidate.condition_number >= 12.0
    assert optimizer.rank([candidate]) == []


def test_rank_prefers_servo_joint_reserve_before_condition_number():
    optimizer = _optimizer(max_condition_number=math.inf)
    low_margin, reserved = _candidates(optimizer)[:2]
    low_margin.visible = True
    low_margin.rejection_reason = ''
    low_margin.condition_number = 5.0
    low_margin.min_joint_margin_rad = 0.20
    reserved.visible = True
    reserved.rejection_reason = ''
    reserved.condition_number = 8.0
    reserved.min_joint_margin_rad = 0.50

    assert optimizer.rank([low_margin, reserved]) == [reserved, low_margin]

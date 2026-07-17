import math

import pytest

from wvcsc_arm_task.observation_optimizer import ObservationOptimizer


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
        'camera_height_min_m': 1.45,
        'camera_height_max_m': 1.75,
        'camera_height_step_m': 0.1,
        'azimuth_offsets_deg': (0.0,),
        'image_margin_ratio': 0.08,
        'max_condition_number': 12.0,
        'min_joint_margin_rad': 0.15,
    }
    config.update(overrides)
    return ObservationOptimizer(
        ROBOT, 'base', 'tip',
        ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'), config)


def _candidates(optimizer):
    return optimizer.generate(
        (2.0, 0.0, 0.0), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (600.0, 600.0, 640.0, 360.0, 1280, 720))


def test_candidates_keep_the_fruit_envelope_inside_camera_margin():
    candidates = _candidates(_optimizer())

    assert candidates
    visible = [candidate for candidate in candidates if candidate.visible]
    assert visible
    assert all(isinstance(candidate.visible, bool) for candidate in candidates)
    assert all(candidate.visible_margin_px >= 0.0 for candidate in visible)


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

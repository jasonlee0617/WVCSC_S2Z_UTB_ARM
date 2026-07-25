import math

import pytest

from wvcsc_mission_manager.core import (
    MissionCore,
    MissionState,
    PointType,
    StopDetector,
    Target,
    docking_pose,
    navigation_pose,
    tree_hint_from_arm_base_offset,
    tree_offset_from_docking,
)


def _targets():
    return [
        Target('tree_1', 3.0, 2.0, 0.0, 0.9, 2.0),
        Target('tree_2', 5.0, -2.0, 0.0, 0.9, 2.0),
    ]


def test_docking_pose_uses_signed_road_normal():
    assert docking_pose(_targets()[0]) == (3.4, 0.2, 0.0)
    assert docking_pose(_targets()[1]) == (5.4, -0.2, 0.0)


def test_docking_pose_supports_explicit_lateral_offset():
    assert docking_pose(_targets()[0], lateral_offset=0.5) == (3.4, 0.5, 0.0)


def test_docking_aligns_arm_base_with_tree_along_road_heading():
    target = _targets()[0]
    goal = docking_pose(target, road_yaw=math.pi / 2.0)
    arm_x = goal[0] - 0.40 * math.cos(goal[2])
    arm_y = goal[1] - 0.40 * math.sin(goal[2])
    tangent_error = (
        (target.x - arm_x) * math.cos(goal[2]) +
        (target.y - arm_y) * math.sin(goal[2]))
    assert tangent_error == pytest.approx(0.0)


def test_rejects_tree_on_road_center_line():
    target = Target('bad', 1.0, 0.0, 0.0, 0.9, 2.0)
    with pytest.raises(ValueError):
        docking_pose(target)


def test_rejects_negative_lateral_offset():
    with pytest.raises(ValueError):
        docking_pose(_targets()[0], lateral_offset=-0.1)


def test_manual_docking_pose_overrides_orchard_offset():
    target = Target(
        'manual_1', 3.0, 2.0, 0.0, 1.0, 2.0,
        docking_pose_override=(3.2, 0.7, 1.2))
    assert navigation_pose(target) == (3.2, 0.7, 1.2)


def test_tree_hint_uses_signed_arm_base_xy_and_round_trips():
    hint = tree_hint_from_arm_base_offset((3.0, 0.5, 0.0), 0.0, -1.5)
    assert hint == (2.6, -1.0, 0.0)
    assert tree_offset_from_docking((3.0, 0.5, 0.0), hint) == pytest.approx(
        (0.0, -1.5))


def test_tree_hint_round_trips_with_alicia_mount_yaw():
    docking = (3.0, 0.5, 0.0)
    hint = tree_hint_from_arm_base_offset(
        docking, 0.0, 1.5,
        arm_base_yaw_rad=math.pi)

    # The actual Alicia installation rotates its base by pi.  Positive arm Y
    # must remain positive when the map hint is transformed back into arm axes.
    assert hint == pytest.approx((2.6, -1.0, 0.0))
    assert tree_offset_from_docking(
        docking, hint, arm_base_yaw_rad=math.pi) == pytest.approx((0.0, 1.5))


def test_docking_pose_aligns_arm_base_with_tree_for_rotated_road():
    target = Target('tree', 3.0, 2.0, 0.0, 0.9, 2.0)
    docking = docking_pose(target, road_yaw=math.pi / 2.0)
    arm_x = docking[0]
    arm_y = docking[1] - 0.40
    assert arm_y == pytest.approx(target.y)


def test_stop_verification_can_return_to_navigation_for_same_target():
    core = MissionCore()
    core.load('retry', [
        Target('tree_1', 3.0, 2.0, 0.0, 0.9, 2.0)])
    assert core.start()
    assert core.nav_succeeded()
    assert core.retry_navigation()
    assert core.state == MissionState.NAVIGATING
    assert core.current_index == 0


def test_two_target_success_path():
    core = MissionCore()
    assert core.load('demo', _targets()) == 'accepted'
    assert core.start()
    assert core.nav_succeeded() and core.stop_verified() and core.arm_succeeded()
    assert core.state == MissionState.NAVIGATING
    assert core.nav_succeeded() and core.stop_verified() and core.arm_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED
    assert core.completed_targets == 2
    assert core.target_outcomes == [core.COMPLETED, core.COMPLETED]


def test_transit_and_finish_points_do_not_send_the_arm_to_spray():
    core = MissionCore()
    core.load('route', [
        Target(
            'transit_01', 0.0, 0.0, 0.0, 1.0, 0.0,
            docking_pose_override=(1.0, 0.0, 0.0),
            point_type=PointType.TRANSIT,
            wide_spray_on_approach=True),
        Target(
            'finish_01', 0.0, 0.0, 0.0, 1.0, 0.0,
            docking_pose_override=(2.0, 0.0, 0.0),
            point_type=PointType.FINISH),
    ])
    assert core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.state == MissionState.DWELLING
    assert core.point_succeeded()
    assert core.state == MissionState.NAVIGATING
    assert core.nav_succeeded() and core.stop_verified()
    assert core.point_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED


def test_optional_return_home_finishes_only_after_home_navigation():
    core = MissionCore()
    core.load('demo', [_targets()[0]])
    core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.arm_succeeded(return_home=True)
    assert core.state == MissionState.RETURNING_HOME
    assert core.home_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED


def test_partial_tree_continues_remaining_targets_and_completes_mission():
    core = MissionCore()
    core.load('demo', _targets())
    core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.arm_partial('sprayed=1 unresolved=1')
    assert core.state == MissionState.NAVIGATING
    assert core.partial_targets == 1
    assert core.completed_targets == 0
    assert core.target_outcomes == [core.PARTIAL, core.PENDING]

    assert core.nav_succeeded() and core.stop_verified() and core.arm_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED
    assert core.completed_targets == 1
    assert core.partial_targets == 1
    assert core.target_outcomes == [core.PARTIAL, core.COMPLETED]


def test_partial_tree_return_home_completes_after_home_navigation():
    core = MissionCore()
    core.load('demo', [_targets()[0]])
    core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.arm_partial('sprayed=1 unresolved=1', return_home=True)
    assert core.state == MissionState.RETURNING_HOME
    assert core.home_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED


def test_safe_skip_and_manual_return_home_semantics():
    core = MissionCore()
    core.load('demo', _targets())
    assert core.skip_current()
    assert core.state == MissionState.READY
    assert core.current_target.tree_id == 'tree_2'
    assert core.skipped_targets == 1
    assert core.target_outcomes == [core.SKIPPED, core.PENDING]
    assert core.return_home()
    assert core.home_succeeded(canceled=True)
    assert core.state == MissionState.CANCELED


def test_completed_mission_can_return_home_without_losing_completion():
    core = MissionCore()
    core.load('demo', [_targets()[0]])
    core.start()
    core.nav_succeeded()
    core.stop_verified()
    core.arm_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED
    assert core.return_home()
    assert core.home_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED


def test_vision_failure_can_safely_skip_after_arm_returns_home():
    core = MissionCore()
    core.load('demo', _targets())
    core.start()
    core.nav_succeeded()
    core.stop_verified()
    assert core.skip_current()
    assert core.current_index == 1
    assert core.state == MissionState.NAVIGATING
    assert core.target_outcomes == [core.SKIPPED, core.PENDING]


def test_failure_marks_only_the_active_target():
    core = MissionCore()
    core.load('demo', _targets())
    core.start()
    core.fail('nav failed')
    assert core.target_outcomes == [core.FAILED, core.PENDING]


def test_duplicate_pause_cancel_and_reset_semantics():
    core = MissionCore()
    assert core.load('demo', _targets()) == 'accepted'
    assert core.load('demo', _targets()) == 'duplicate'
    assert core.start() and core.pause() and core.resume()
    assert core.cancel()
    assert core.reset()
    assert core.state == MissionState.WAITING_FOR_TASKS


def test_recovery_pause_resumes_navigation_or_home_intent():
    core = MissionCore()
    assert core.load('demo', _targets()) == 'accepted'
    assert core.start()
    assert core.pause_for_recovery()
    assert core.resume()
    assert core.state == MissionState.NAVIGATING

    assert core.pause()
    assert core.return_home()
    assert core.pause_for_recovery()
    assert core.resume(returning_home=True)
    assert core.state == MissionState.RETURNING_HOME


def test_failure_is_terminal_and_does_not_advance_target():
    core = MissionCore()
    core.load('demo', _targets())
    core.start()
    assert core.fail('nav failed')
    assert not core.nav_succeeded()
    assert core.current_index == 0
    assert core.state == MissionState.FAILED


def test_stop_detector_requires_continuous_fresh_samples():
    detector = StopDetector(stable_duration=1.0, stale_timeout=0.5, timeout=5.0)
    detector.start(0.0)
    detector.update(0.1, 0.0, 0.0)
    detector.update(0.6, 0.1, 0.0)
    detector.update(0.7, 0.0, 0.0)
    detector.update(1.2, 0.0, 0.0)
    assert detector.status(1.6) == StopDetector.WAITING
    detector.update(1.7, 0.0, 0.0)
    assert detector.status(1.7) == StopDetector.STABLE


def test_stop_detector_reports_stale_and_timeout():
    detector = StopDetector(stale_timeout=1.0, timeout=5.0)
    detector.start(0.0)
    assert detector.status(1.0) == StopDetector.STALE
    assert detector.status(5.0) == StopDetector.TIMEOUT


def test_stop_detector_times_out_when_vehicle_never_stops():
    detector = StopDetector(stale_timeout=1.0, timeout=5.0)
    detector.start(0.0)
    for timestamp in (0.5, 1.5, 2.5, 3.5, 4.5):
        detector.update(timestamp, 0.1, 0.1)
    assert detector.status(5.0) == StopDetector.TIMEOUT


def test_stale_odom_cannot_be_mistaken_for_continuous_stability():
    detector = StopDetector(
        stable_duration=1.0, stale_timeout=0.5, timeout=5.0)
    detector.start(0.0)
    detector.update(0.1, 0.0, 0.0)
    assert detector.status(1.1) == StopDetector.STALE

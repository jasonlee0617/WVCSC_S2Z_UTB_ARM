import math

import pytest

from wvcsc_mission_manager.core import (
    MissionCore,
    MissionState,
    PointType,
    RoutePoint,
    tree_hint_from_arm_base_offset,
)
from wvcsc_mission_manager.stop_detector import StopDetector


def _points():
    return [
        RoutePoint('point_1', 3.0, 2.0, 0.0, 0.9, (3.4, 0.2, 0.0)),
        RoutePoint('point_2', 5.0, -2.0, 0.0, 0.9, (5.4, -0.2, 0.0)),
    ]


def test_manual_task_keeps_the_operator_selected_vehicle_docking_pose():
    assert _points()[0].docking_pose == (3.4, 0.2, 0.0)
    assert _points()[1].docking_pose == (5.4, -0.2, 0.0)


def test_tree_hint_uses_signed_arm_base_offset():
    hint = tree_hint_from_arm_base_offset((3.0, 0.5, 0.0), 0.0, -1.5)
    assert hint == (2.6, -1.0, 0.0)


def test_tree_hint_round_trips_with_alicia_mount_yaw():
    docking = (3.0, 0.5, 0.0)
    hint = tree_hint_from_arm_base_offset(
        docking, 0.0, 1.5,
        arm_base_yaw_rad=math.pi)

    # The actual Alicia installation rotates its base by pi.
    assert hint == pytest.approx((2.6, -1.0, 0.0))


def test_two_point_success_path():
    core = MissionCore()
    assert core.load('demo', _points()) == 'accepted'
    assert core.start()
    assert core.nav_succeeded() and core.stop_verified() and core.arm_succeeded()
    assert core.state == MissionState.NAVIGATING
    assert core.nav_succeeded() and core.stop_verified() and core.arm_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED
    assert core.completed_points == 2
    assert core.point_outcomes == [core.COMPLETED, core.COMPLETED]


def test_transit_points_do_not_send_the_arm_to_spray():
    core = MissionCore()
    core.load('route', [
        RoutePoint(
            'transit_01', 0.0, 0.0, 0.0, 1.0, (1.0, 0.0, 0.0),
            point_type=PointType.TRANSIT,
            wide_spray_on_approach=True),
        RoutePoint(
            'transit_02', 0.0, 0.0, 0.0, 1.0, (2.0, 0.0, 0.0),
            point_type=PointType.TRANSIT),
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
    core.load('demo', [_points()[0]])
    core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.arm_succeeded(return_home=True)
    assert core.state == MissionState.RETURNING_HOME
    assert core.home_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED


def test_partial_point_continues_remaining_points_and_completes_mission():
    core = MissionCore()
    core.load('demo', _points())
    core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.arm_partial('sprayed=1 unresolved=1')
    assert core.state == MissionState.NAVIGATING
    assert core.partial_points == 1
    assert core.completed_points == 0
    assert core.point_outcomes == [core.PARTIAL, core.PENDING]

    assert core.nav_succeeded() and core.stop_verified() and core.arm_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED
    assert core.completed_points == 1
    assert core.partial_points == 1
    assert core.point_outcomes == [core.PARTIAL, core.COMPLETED]


def test_partial_point_return_home_completes_after_home_navigation():
    core = MissionCore()
    core.load('demo', [_points()[0]])
    core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.arm_partial('sprayed=1 unresolved=1', return_home=True)
    assert core.state == MissionState.RETURNING_HOME
    assert core.home_succeeded()
    assert core.state == MissionState.MISSION_COMPLETED


def test_safe_skip_and_manual_return_home_semantics():
    core = MissionCore()
    core.load('demo', _points())
    assert core.skip_current()
    assert core.state == MissionState.READY
    assert core.current_point.point_id == 'point_2'
    assert core.skipped_points == 1
    assert core.point_outcomes == [core.SKIPPED, core.PENDING]
    assert core.return_home()
    assert core.home_succeeded(canceled=True)
    assert core.state == MissionState.CANCELED


def test_completed_mission_can_return_home_without_losing_completion():
    core = MissionCore()
    core.load('demo', [_points()[0]])
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
    core.load('demo', _points())
    core.start()
    core.nav_succeeded()
    core.stop_verified()
    assert core.skip_current()
    assert core.current_index == 1
    assert core.state == MissionState.NAVIGATING
    assert core.point_outcomes == [core.SKIPPED, core.PENDING]


def test_failure_marks_only_the_active_point():
    core = MissionCore()
    core.load('demo', _points())
    core.start()
    core.fail('nav failed')
    assert core.point_outcomes == [core.FAILED, core.PENDING]


def test_duplicate_pause_cancel_and_reset_semantics():
    core = MissionCore()
    assert core.load('demo', _points()) == 'accepted'
    assert core.load('demo', _points()) == 'duplicate'
    assert core.start() and core.pause() and core.resume()
    assert core.cancel()
    assert core.reset()
    assert core.state == MissionState.WAITING_FOR_TASKS


def test_recovery_pause_resumes_navigation_or_home_intent():
    core = MissionCore()
    assert core.load('demo', _points()) == 'accepted'
    assert core.start()
    assert core.pause_for_recovery()
    assert core.resume()
    assert core.state == MissionState.NAVIGATING

    assert core.pause()
    assert core.return_home()
    assert core.pause_for_recovery()
    assert core.resume(returning_home=True)
    assert core.state == MissionState.RETURNING_HOME


def test_failure_is_terminal_and_does_not_advance_point():
    core = MissionCore()
    core.load('demo', _points())
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


def test_stop_detector_can_override_duration_for_one_check():
    detector = StopDetector(stable_duration=1.0, stale_timeout=1.0, timeout=5.0)
    detector.start(0.0, stable_duration=0.2)
    detector.update(0.1, 0.0, 0.0)
    assert detector.status(0.29) == StopDetector.WAITING
    detector.update(0.3, 0.0, 0.0)
    assert detector.status(0.5) == StopDetector.STABLE


def test_stop_detector_default_duration_is_preserved():
    detector = StopDetector(stable_duration=1.0, stale_timeout=1.0, timeout=5.0)
    detector.start(0.0, stable_duration=0.2)
    detector.update(0.1, 0.0, 0.0)
    detector.update(0.2, 0.0, 0.0)
    assert detector.status(0.4) == StopDetector.STABLE
    detector.stop()
    detector.start(1.0)
    detector.update(1.1, 0.0, 0.0)
    assert detector.status(1.5) == StopDetector.WAITING


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

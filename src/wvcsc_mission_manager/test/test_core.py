import pytest

from wvcsc_mission_manager.core import (
    MissionCore,
    MissionState,
    StopDetector,
    Target,
    docking_pose,
)


def _targets():
    return [
        Target('tree_1', 3.0, 2.0, 0.0, 0.9, 'left', 2.0),
        Target('tree_2', 5.0, -2.0, 0.0, 0.9, 'right', 2.0),
    ]


def test_docking_pose_uses_tree_coordinate_and_side():
    assert docking_pose(_targets()[0]) == (3.0, 0.5, 0.0)
    assert docking_pose(_targets()[1]) == (5.0, -0.5, 0.0)


def test_rejects_side_that_disagrees_with_road_geometry():
    target = Target('bad', 1.0, -2.0, 0.0, 0.9, 'left', 2.0)
    with pytest.raises(ValueError):
        docking_pose(target)


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


def test_optional_return_home_finishes_only_after_home_navigation():
    core = MissionCore()
    core.load('demo', [_targets()[0]])
    core.start()
    assert core.nav_succeeded() and core.stop_verified()
    assert core.arm_succeeded(return_home=True)
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

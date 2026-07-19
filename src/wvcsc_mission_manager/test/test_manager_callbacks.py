import math
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from wvcsc_interfaces.action import ExecuteSpray

from wvcsc_mission_manager.core import (
    MissionCore,
    MissionState,
    StopDetector,
    Target,
)
from wvcsc_mission_manager.mission_manager import MissionManager


class _Future:
    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result


class _Logger:
    def debug(self, _message):
        pass

    def error(self, _message):
        pass

    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class _Detector:
    def __init__(self, status=StopDetector.WAITING):
        self.value = status
        self.stopped = False

    def status(self, _now):
        return self.value

    def stop(self):
        self.stopped = True


class _NavClient:
    def __init__(self, ready=True):
        self.ready = ready

    def server_is_ready(self):
        return self.ready


class _Harness:
    def __init__(self, state=MissionState.NAVIGATING):
        target = Target('tree_1', 3.0, 2.0, 0.0, 0.9, 'left', 2.0)
        self.core = MissionCore()
        self.core.load('demo', [target])
        self.core.state = state
        self._nav_pending = True
        self._spray_pending = True
        self._nav_handle = object()
        self._spray_handle = object()
        self._phase_started = 0.0
        self._nav_timeout = 5.0
        self._nav_startup_retry_timeout = 30.0
        self._nav_startup_retry_interval = 0.5
        self._initial_nav_started = None
        self._nav_retry_due = None
        self._spray_timeout = 5.0
        self._spray_progress_timeout = 3.0
        self._spray_last_progress = 0.0
        self._auto_start = False
        self._return_home_after_finish = False
        self._manual_return_home = False
        self._home_pose = (0.0, 0.0, 0.0)
        self._stop_detector = _Detector()
        self._nav_client = _NavClient()
        self.failures = []
        self.nav_cancels = 0
        self.spray_cancels = 0
        self.status_updates = 0
        self._navigation_active = MissionManager._navigation_active.__get__(
            self, _Harness)
        self._clear_nav_startup_retry = (
            MissionManager._clear_nav_startup_retry.__get__(self, _Harness))
        self._schedule_initial_nav_retry = (
            MissionManager._schedule_initial_nav_retry.__get__(self, _Harness))

    def _fail(self, message):
        self.failures.append(str(message))
        self.core.fail(message)

    def _publish_status(self):
        self.status_updates += 1

    def _cancel_nav_goal(self):
        self.nav_cancels += 1

    def _cancel_spray_goal(self):
        self.spray_cancels += 1

    def _now(self):
        return 6.0

    def get_logger(self):
        return _Logger()


class _Validator:
    def __init__(self):
        self._map_frame = 'map'
        self._road_center_y = 0.0
        self._road_yaw = 0.0
        self._docking_lateral_offset = 0.5
        self.parameters = {
            'confidence_threshold': 0.5,
            'max_targets': 2,
            'max_abs_coordinate': 50.0,
            'min_spray_duration': 0.2,
            'max_spray_duration': 10.0,
        }

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])


def _pose(x, y, yaw=0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)),
    )


def _manual_request(frame='map', x=3.0, count=1):
    targets = [SimpleNamespace(
        target_id=f'manual_{index}',
        docking_pose=_pose(x + index, 0.5, yaw=0.2 + index),
        spray_side='left',
        spray_duration=2.0,
    ) for index in range(count)]
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        mission_id='manual_demo',
        home_pose=_pose(0.0, 0.0),
        return_home_after_finish=False,
        targets=targets,
    )


def _message(frame='map', x=3.0, count=1):
    trees = [SimpleNamespace(
        tree_id=f'tree_{index}',
        position=SimpleNamespace(x=x, y=2.0, z=0.0),
        confidence=0.9,
        spray_side='left',
        spray_duration=2.0,
        evidence_uri='',
    ) for index in range(count)]
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        mission_id='demo',
        source_mode='mock',
        trees=trees,
    )


def test_nav_rejection_and_failure_are_fail_fast():
    rejected = _Harness()
    MissionManager._nav_goal_response(
        rejected, _Future(SimpleNamespace(accepted=False)))
    assert rejected.core.state == MissionState.FAILED
    assert rejected.failures == ['Nav2 rejected the goal']

    failed = _Harness()
    MissionManager._nav_result(
        failed,
        _Future(SimpleNamespace(status=GoalStatus.STATUS_ABORTED)),
    )
    assert failed.core.state == MissionState.FAILED
    assert failed.core.current_index == 0


def test_initial_nav_rejection_retries_while_lifecycle_activates():
    retrying = _Harness()
    retrying._initial_nav_started = 0.0
    retrying._now = lambda: 1.0

    MissionManager._nav_goal_response(
        retrying, _Future(SimpleNamespace(accepted=False)))

    assert retrying.core.state == MissionState.NAVIGATING
    assert retrying._nav_retry_due == 1.5
    assert retrying.failures == []


def test_paused_navigation_cancel_does_not_fail_mission():
    paused = _Harness(MissionState.PAUSED)
    MissionManager._nav_result(
        paused,
        _Future(SimpleNamespace(status=GoalStatus.STATUS_CANCELED)),
    )
    assert paused.core.state == MissionState.PAUSED
    assert paused.failures == []
    assert paused._nav_handle is None


def test_spray_rejection_and_failed_result_do_not_advance_target():
    rejected = _Harness(MissionState.ARM_SPRAYING)
    MissionManager._spray_goal_response(
        rejected, _Future(SimpleNamespace(accepted=False)))
    assert rejected.core.state == MissionState.FAILED
    assert rejected.core.current_index == 0

    failed = _Harness(MissionState.ARM_SPRAYING)
    result = SimpleNamespace(
        success=False, error_code=6, message='HOME motion failed')
    MissionManager._spray_result(
        failed,
        _Future(SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED, result=result)),
    )
    assert failed.core.state == MissionState.FAILED
    assert failed.core.current_index == 0


def test_ordinary_vision_failure_skips_target_after_home():
    harness = _Harness(MissionState.ARM_SPRAYING)
    result = SimpleNamespace(
        success=False,
        error_code=ExecuteSpray.Result.VISION_FAILED,
        message='target unavailable/stale',
    )

    MissionManager._spray_result(
        harness,
        _Future(SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED, result=result)),
    )

    assert harness.core.state == MissionState.FAILED
    assert harness.core.skipped_targets == 1
    assert harness.core.target_outcomes == [MissionCore.SKIPPED]
    assert harness.failures == []


def test_inspected_without_disease_completes_the_tree():
    harness = _Harness(MissionState.ARM_SPRAYING)
    result = SimpleNamespace(
        success=True,
        error_code=ExecuteSpray.Result.INSPECTED_NO_DISEASE,
        message='tree inspected; no diseased fruit detected',
    )

    MissionManager._spray_result(
        harness,
        _Future(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=result)),
    )

    assert harness.core.state == MissionState.MISSION_COMPLETED
    assert harness.core.completed_targets == 1
    assert harness.core.skipped_targets == 0


def test_partial_success_marks_tree_incomplete_and_fails_final_mission():
    harness = _Harness(MissionState.ARM_SPRAYING)
    result = SimpleNamespace(
        success=True,
        error_code=ExecuteSpray.Result.PARTIAL_SUCCESS,
        message='sprayed=1 skipped=1',
    )

    MissionManager._spray_result(
        harness,
        _Future(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=result)),
    )

    assert harness.core.state == MissionState.FAILED
    assert harness.core.completed_targets == 0
    assert harness.core.partial_targets == 1
    assert harness.core.target_outcomes == [MissionCore.PARTIAL]


def test_nav_spray_and_stop_timeouts_fail_the_active_target():
    navigating = _Harness(MissionState.NAVIGATING)
    MissionManager._tick(navigating)
    assert navigating.failures == ['Nav2 goal timed out']

    spraying = _Harness(MissionState.ARM_SPRAYING)
    MissionManager._tick(spraying)
    assert spraying.failures == ['spray Action timed out']

    stale = _Harness(MissionState.VERIFYING_STOP)
    stale._stop_detector = _Detector(StopDetector.STALE)
    MissionManager._tick(stale)
    assert stale.failures == ['odom stop verification failed: stale']


def test_spray_feedback_prevents_the_progress_watchdog_from_canceling_work():
    harness = _Harness(MissionState.ARM_SPRAYING)
    harness._spray_timeout = 10.0
    harness._spray_progress_timeout = 3.0
    harness._spray_last_progress = 4.0

    MissionManager._tick(harness)

    assert harness.failures == []
    harness._spray_last_progress = 2.0
    MissionManager._tick(harness)
    assert harness.failures == ['spray Action made no progress']


def test_fail_cancels_both_children_and_stops_odom_check():
    harness = _Harness(MissionState.ARM_SPRAYING)
    harness._fail = MissionManager._fail.__get__(harness, _Harness)
    harness._publish_status = lambda: setattr(
        harness, 'status_updates', harness.status_updates + 1)

    harness._fail('forced failure')

    assert harness.core.state == MissionState.FAILED
    assert harness._stop_detector.stopped
    assert harness.nav_cancels == 1
    assert harness.spray_cancels == 1
    assert harness.status_updates == 1


def test_invalid_frame_coordinate_and_target_count_are_rejected():
    validator = _Validator()
    for message in (
            _message(frame='odom'),
            _message(x=51.0),
            _message(count=3)):
        try:
            MissionManager._validate_message(validator, message)
        except ValueError:
            continue
        raise AssertionError('invalid mission was accepted')

    invalid_source = _message()
    invalid_source.source_mode = 'serial'
    try:
        MissionManager._validate_message(validator, invalid_source)
    except ValueError:
        pass
    else:
        raise AssertionError('invalid source_mode was accepted')


def test_manual_targets_preserve_selected_pose_and_validate_input():
    validator = _Validator()
    request = _manual_request()
    targets, home = MissionManager._validate_manual_request(validator, request)
    assert targets[0].docking_pose_override[:2] == (3.0, 0.5)
    assert math.isclose(targets[0].docking_pose_override[2], 0.2)
    assert home == (0.0, 0.0, 0.0)

    for invalid in (
            _manual_request(frame='odom'),
            _manual_request(x=51.0),
            _manual_request(count=3)):
        try:
            MissionManager._validate_manual_request(validator, invalid)
        except ValueError:
            continue
        raise AssertionError('invalid manual mission was accepted')


def test_start_rejects_when_action_servers_are_absent():
    harness = _Harness(MissionState.READY)
    harness._servers_ready = lambda: False
    harness._reply = MissionManager._reply
    response = SimpleNamespace(success=None, message='')

    MissionManager._start(harness, None, response)

    assert not response.success
    assert harness.core.state == MissionState.READY


def test_cancel_stops_both_active_children():
    harness = _Harness(MissionState.ARM_SPRAYING)
    harness._reply = MissionManager._reply
    response = SimpleNamespace(success=None, message='')

    MissionManager._cancel(harness, None, response)

    assert response.success
    assert harness.core.state == MissionState.CANCELED
    assert harness._stop_detector.stopped
    assert harness.nav_cancels == 1
    assert harness.spray_cancels == 1


def test_last_target_can_require_return_home_navigation():
    harness = _Harness(MissionState.ARM_SPRAYING)
    harness._return_home_after_finish = True
    harness._send_nav_goal = lambda: setattr(harness, 'nav_sent', True)
    result = SimpleNamespace(success=True, error_code=0, message='HOME')

    MissionManager._spray_result(
        harness,
        _Future(SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED, result=result)),
    )

    assert harness.core.state == MissionState.RETURNING_HOME
    assert harness.nav_sent


def test_return_home_nav_success_completes_mission():
    harness = _Harness(MissionState.RETURNING_HOME)
    harness.core.target_outcomes = [MissionCore.COMPLETED]
    harness.core.completed_targets = 1

    MissionManager._nav_result(
        harness,
        _Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)),
    )

    assert harness.core.state == MissionState.MISSION_COMPLETED


def test_manual_return_home_nav_success_cancels_remaining_mission():
    harness = _Harness(MissionState.RETURNING_HOME)
    harness._manual_return_home = True

    MissionManager._nav_result(
        harness,
        _Future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED)),
    )

    assert harness.core.state == MissionState.CANCELED


def test_skip_requires_paused_navigation_to_settle():
    harness = _Harness(MissionState.PAUSED)
    harness._reply = MissionManager._reply
    response = SimpleNamespace(success=None, message='')

    MissionManager._skip_current(harness, None, response)
    assert not response.success
    assert harness.core.skipped_targets == 0

    harness._nav_handle = None
    harness._nav_pending = False
    MissionManager._skip_current(harness, None, response)
    assert response.success
    assert harness.core.skipped_targets == 1
    assert harness.core.state == MissionState.FAILED


def test_manual_return_home_is_started_only_from_safe_settled_state():
    harness = _Harness(MissionState.READY)
    harness._reply = MissionManager._reply
    harness._nav_handle = None
    harness._spray_handle = None
    harness._nav_pending = False
    harness._spray_pending = False
    harness._send_nav_goal = lambda: setattr(harness, 'nav_sent', True)
    response = SimpleNamespace(success=None, message='')

    MissionManager._return_home(harness, None, response)

    assert response.success
    assert harness._manual_return_home
    assert harness.core.state == MissionState.RETURNING_HOME
    assert harness.nav_sent

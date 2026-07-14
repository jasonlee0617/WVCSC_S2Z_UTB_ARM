from types import SimpleNamespace

from action_msgs.msg import GoalStatus

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
    def error(self, _message):
        pass


class _Detector:
    def __init__(self, status=StopDetector.WAITING):
        self.value = status
        self.stopped = False

    def status(self, _now):
        return self.value

    def stop(self):
        self.stopped = True


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
        self._spray_timeout = 5.0
        self._auto_start = False
        self._stop_detector = _Detector()
        self.failures = []
        self.nav_cancels = 0
        self.spray_cancels = 0
        self.status_updates = 0

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
        self._standoff = 1.5
        self.parameters = {
            'confidence_threshold': 0.5,
            'max_targets': 2,
            'max_abs_coordinate': 50.0,
            'min_spray_duration': 0.2,
            'max_spray_duration': 10.0,
        }

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])


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

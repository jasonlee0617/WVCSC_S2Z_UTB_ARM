import math
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from wvcsc_interfaces.action import ExecuteSpray

from wvcsc_mission_manager.core import (
    MissionCore,
    MissionState,
    PointType,
    RoutePoint,
)
from wvcsc_mission_manager.mission_manager import MissionManager
from wvcsc_mission_manager.stop_detector import StopDetector


class _Future:
    def __init__(self, result=None, error=None):
        self._result = result
        self._error = error

    def result(self):
        if self._error is not None:
            raise self._error
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class _Logger:
    def debug(self, _message):
        pass

    def error(self, _message):
        pass

    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def warning(self, _message):
        pass


class _Detector:
    def __init__(self, status=StopDetector.WAITING):
        self.value = status
        self.stopped = False
        self.started = None

    def start(self, now, stable_duration=None):
        self.started = (float(now), stable_duration)

    def status(self, _now):
        return self.value

    def stop(self):
        self.stopped = True


class _NavClient:
    def __init__(self, ready=True):
        self.ready = ready

    def server_is_ready(self):
        return self.ready


class _Relay:
    def __init__(self):
        self.commands = []
        self.reset_count = 0
        self.wide_enabled = False

    @staticmethod
    def service_is_ready():
        return True

    def command(self, channel, enabled, duration, continuation, context, **_kwargs):
        self.commands.append((channel, enabled, duration, context))
        if channel == 1:
            self.wide_enabled = bool(enabled)
        if continuation is not None:
            continuation()

    def command_all_off(self):
        self.commands.extend(((1, False, 0.0, 'mission shutdown: disable wide spray'),
                              (2, False, 0.0, 'mission shutdown: disable arm spray')))

    def reset_failure_latch(self):
        self.reset_count += 1


class _WideMotionHarness:
    def __init__(self):
        point = RoutePoint(
            'transit_1', 3.0, 2.0, 0.0, 0.0, (3.4, 0.2, 0.0),
            point_type=PointType.TRANSIT, wide_spray_on_approach=True)
        self.core = MissionCore()
        self.core.load('wide_motion', [point])
        self.core.state = MissionState.NAVIGATING
        self._wide_motion_pending = True
        self._wide_motion_deadline = 10.0
        self._last_odom_at = 5.0
        self._latest_linear_speed = 0.0
        self._latest_angular_speed = 0.0
        self._wide_motion_linear_threshold = 0.03
        self._linear_stop_threshold = 0.03
        self._angular_stop_threshold = 0.03
        self._stop_stable_duration = 1.0
        self._odom_stale_timeout = 1.0
        self._wide_stop_started_at = None
        self._wide_stop_off_pending = False
        self._wide_relay_channel = 1
        self._relay = _Relay()
        self.commands = self._relay.commands
        self._reset_wide_spray_motion = (
            MissionManager._reset_wide_spray_motion.__get__(self, _WideMotionHarness))
        self._disable_wide_spray_after_stop = (
            MissionManager._disable_wide_spray_after_stop.__get__(self, _WideMotionHarness))

    @staticmethod
    def get_logger():
        return _Logger()

class _Harness:
    def __init__(self, state=MissionState.NAVIGATING):
        point = RoutePoint('point_1', 3.0, 2.0, 0.0, 0.9, (3.4, 0.2, 0.0))
        self.core = MissionCore()
        self.core.load('demo', [point])
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
        self._stop_stable_duration = 1.0
        self._transit_stop_stable_duration = 0.2
        self._return_home_after_mission = False
        self._manual_return_home = False
        self._home_pose = (0.0, 0.0, 0.0)
        self._stop_detector = _Detector()
        self._nav_client = _NavClient()
        self._abort_and_home_requested = False
        self._abort_reset_sent = False
        self._abort_reset_started = False
        self._motion_control_state = ''
        self._recovery_return_home = False
        self._nav_timeout_canceling = False
        self._nav_timeout_cancel_deadline = None
        self._wide_motion_pending = False
        self._wide_motion_deadline = None
        self._wide_stop_started_at = None
        self._wide_stop_off_pending = False
        self._wide_relay_channel = 1
        self.failures = []
        self._relay = _Relay()
        self.relay_commands = self._relay.commands
        self.nav_cancels = 0
        self.spray_cancels = 0
        self.status_updates = 0
        self._navigation_active = MissionManager._navigation_active.__get__(
            self, _Harness)
        self._nav_result = MissionManager._nav_result.__get__(self, _Harness)
        self._navigation_arrived = (
            MissionManager._navigation_arrived.__get__(self, _Harness))
        self._reset_wide_spray_motion = (
            MissionManager._reset_wide_spray_motion.__get__(self, _Harness))
        self._clear_nav_startup_retry = (
            MissionManager._clear_nav_startup_retry.__get__(self, _Harness))
        self._schedule_initial_nav_retry = (
            MissionManager._schedule_initial_nav_retry.__get__(self, _Harness))
        self._skip_navigation_point = (
            MissionManager._skip_navigation_point.__get__(self, _Harness))
        self._skip_arm_point = (
            MissionManager._skip_arm_point.__get__(self, _Harness))
        self._continue_after_point = (
            MissionManager._continue_after_point.__get__(self, _Harness))
        self._finish_noninspect_point = (
            MissionManager._finish_noninspect_point.__get__(self, _Harness))
        self._advance_abort_and_home = lambda: None
        self._command_all_relays_off = lambda: None
        self._start_navigation = lambda: setattr(self, 'nav_sent', True)
        self._begin_stop_verification = lambda: None
        self._tick_wide_spray_motion = lambda _now: None
        self._tick_startup_retry = (
            MissionManager._tick_startup_retry.__get__(self, _Harness))
        self._tick_navigation_timeout = (
            MissionManager._tick_navigation_timeout.__get__(self, _Harness))
        self._tick_active_work_phase = (
            MissionManager._tick_active_work_phase.__get__(self, _Harness))

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
        self._arm_base_forward_offset = -0.40
        self._arm_base_left_offset = 0.0
        self._arm_base_yaw = 0.0
        self.parameters = {
            'max_points': 2,
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
    points = [SimpleNamespace(
        point_id=f'manual_{index}',
        docking_pose=_pose(x + index, 0.5, yaw=0.2 + index),
        spray_duration=2.0,
        tree_x_m=0.0,
        tree_y_m=1.5,
        tree_base_z_m=0.0,
    ) for index in range(count)]
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame),
        mission_id='manual_demo',
        home_pose=_pose(0.0, 0.0),
        return_home_after_mission=False,
        points=points,
    )


def test_nav_rejection_and_confirmed_failure_skip_current_route_point():
    rejected = _Harness()
    MissionManager._nav_goal_response(
        rejected, _Future(SimpleNamespace(accepted=False)))
    assert rejected.core.state == MissionState.MISSION_COMPLETED
    assert rejected.core.skipped_points == 1
    assert rejected.failures == []

    failed = _Harness()
    MissionManager._nav_result(
        failed,
        _Future(SimpleNamespace(status=GoalStatus.STATUS_ABORTED)),
    )
    assert failed.core.state == MissionState.MISSION_COMPLETED
    assert failed.core.current_index == 1
    assert failed.relay_commands[0][:3] == (1, False, 0.0)


def test_transit_arrival_disables_wide_spray_before_any_dwell_or_next_point():
    harness = _Harness()
    transit = RoutePoint(
        'transit_1', 3.0, 2.0, 0.0, 0.0, (3.4, 0.2, 0.0),
        point_type=PointType.TRANSIT, wide_spray_on_approach=True,
        dwell_time_sec=1.0)
    harness.core = MissionCore()
    harness.core.load('transit_demo', [transit])
    harness.core.state = MissionState.NAVIGATING

    MissionManager._navigation_arrived(harness)

    assert harness.core.state == MissionState.VERIFYING_STOP
    assert harness.relay_commands == [
        (1, False, 0.0, 'transit_1: disable wide spray at stop')]


def test_stop_verification_uses_short_duration_for_transit_points():
    harness = _Harness(state=MissionState.VERIFYING_STOP)
    transit = RoutePoint(
        'transit_1', 3.0, 2.0, 0.0, 0.0, (3.4, 0.2, 0.0),
        point_type=PointType.TRANSIT)
    harness.core = MissionCore()
    harness.core.load('transit_demo', [transit])
    harness.core.state = MissionState.VERIFYING_STOP

    MissionManager._begin_stop_verification(harness)

    assert harness._stop_detector.started == (6.0, 0.2)


def test_zero_transit_stop_confirmation_skips_detector_after_wide_off():
    harness = _Harness()
    harness._transit_stop_stable_duration = 0.0
    transit = RoutePoint(
        'transit_1', 3.0, 2.0, 0.0, 0.0, (3.4, 0.2, 0.0),
        point_type=PointType.TRANSIT, wide_spray_on_approach=True,
        dwell_time_sec=1.0)
    following = RoutePoint(
        'transit_2', 4.0, 2.0, 0.0, 0.0, (4.4, 0.2, 0.0),
        point_type=PointType.TRANSIT)
    harness.core = MissionCore()
    harness.core.load('zero_transit_demo', [transit, following])
    harness.core.state = MissionState.NAVIGATING
    harness._begin_stop_verification = (
        MissionManager._begin_stop_verification.__get__(harness, _Harness))

    MissionManager._navigation_arrived(harness)

    assert harness.relay_commands == [
        (1, False, 0.0, 'transit_1: disable wide spray at stop')]
    assert harness._stop_detector.started is None
    assert harness.core.state == MissionState.DWELLING
    assert harness._phase_started == 6.0
    assert not hasattr(harness, 'nav_sent')

    MissionManager._tick_active_work_phase(harness, 6.9)
    assert harness.core.state == MissionState.DWELLING

    MissionManager._tick_active_work_phase(harness, 7.0)
    assert harness.core.state == MissionState.NAVIGATING
    assert harness.nav_sent


def test_stop_verification_keeps_arm_duration_for_inspect_points():
    harness = _Harness(state=MissionState.VERIFYING_STOP)
    harness._transit_stop_stable_duration = 0.0
    inspect = RoutePoint(
        'inspect_1', 3.0, 2.0, 0.0, 0.9, (3.4, 0.2, 0.0),
        point_type=PointType.INSPECT)
    harness.core = MissionCore()
    harness.core.load('inspect_demo', [inspect])
    harness.core.state = MissionState.VERIFYING_STOP

    MissionManager._begin_stop_verification(harness)

    assert harness._stop_detector.started == (6.0, 1.0)


def test_wide_spray_waits_for_motion_and_times_out_with_relay_off():
    harness = _WideMotionHarness()
    MissionManager._tick_wide_spray_motion(harness, now=5.0)
    assert harness.commands == []
    assert harness._wide_motion_pending is True

    harness._latest_linear_speed = 0.03
    MissionManager._tick_wide_spray_motion(harness, now=5.0)
    assert harness.commands == [
        (1, True, 0.0,
         'transit_1: vehicle motion confirmed; enable wide spray')]
    assert harness._wide_motion_pending is False

    timeout = _WideMotionHarness()
    MissionManager._tick_wide_spray_motion(timeout, now=10.0)
    assert timeout.commands == []
    assert timeout._wide_motion_pending is False


def test_wide_spray_keeps_running_through_brief_or_interrupted_stops():
    harness = _WideMotionHarness()
    harness._wide_motion_pending = False
    harness._wide_motion_deadline = None
    harness._relay.wide_enabled = True
    harness._latest_linear_speed = 0.04

    MissionManager._tick_wide_spray_motion(harness, now=5.0)
    assert harness.commands == []
    assert harness._wide_stop_started_at is None

    harness._latest_linear_speed = 0.0

    MissionManager._tick_wide_spray_motion(harness, now=5.0)
    MissionManager._tick_wide_spray_motion(harness, now=5.9)
    assert harness.commands == []

    harness._latest_linear_speed = 0.04
    MissionManager._tick_wide_spray_motion(harness, now=5.95)
    assert harness._wide_stop_started_at is None
    assert harness.commands == []


def test_wide_spray_turns_off_once_after_stable_stop_and_reopens_on_motion():
    harness = _WideMotionHarness()
    harness._wide_motion_pending = False
    harness._wide_motion_deadline = None
    harness._relay.wide_enabled = True
    harness._latest_linear_speed = 0.0

    MissionManager._tick_wide_spray_motion(harness, now=5.0)
    MissionManager._tick_wide_spray_motion(harness, now=6.0)
    assert harness.commands == [
        (1, False, 0.0,
         'transit_1: vehicle remained stopped; disable wide spray')]
    assert harness._wide_motion_pending is True
    assert harness._relay.wide_enabled is False

    MissionManager._tick_wide_spray_motion(harness, now=6.1)
    assert len(harness.commands) == 1

    harness._latest_linear_speed = 0.03
    harness._last_odom_at = 6.1
    MissionManager._tick_wide_spray_motion(harness, now=6.1)
    assert harness.commands[-1] == (
        1, True, 0.0,
        'transit_1: vehicle motion confirmed; enable wide spray')
    assert harness._relay.wide_enabled is True


def test_wide_spray_turns_off_immediately_when_odometry_becomes_stale():
    harness = _WideMotionHarness()
    harness._wide_motion_pending = False
    harness._wide_motion_deadline = None
    harness._relay.wide_enabled = True
    harness._last_odom_at = 4.0

    MissionManager._tick_wide_spray_motion(harness, now=5.1)
    assert harness.commands == [
        (1, False, 0.0,
         'transit_1: odometry became stale; disable wide spray')]
    assert harness._wide_motion_pending is True


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


def test_spray_rejection_skips_but_home_failure_remains_blocking():
    rejected = _Harness(MissionState.ARM_SPRAYING)
    MissionManager._spray_goal_response(
        rejected, _Future(SimpleNamespace(accepted=False)))
    assert rejected.core.state == MissionState.MISSION_COMPLETED
    assert rejected.core.current_index == 1

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


def test_ordinary_vision_failure_skips_point_after_home():
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

    assert harness.core.state == MissionState.MISSION_COMPLETED
    assert harness.core.skipped_points == 1
    assert harness.core.point_outcomes == [MissionCore.SKIPPED]
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
    assert harness.core.completed_points == 1
    assert harness.core.skipped_points == 0


def test_partial_success_marks_tree_incomplete_but_completes_route():
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

    assert harness.core.state == MissionState.MISSION_COMPLETED
    assert harness.core.completed_points == 0
    assert harness.core.partial_points == 1
    assert harness.core.point_outcomes == [MissionCore.PARTIAL]


def test_nav_timeout_cancels_for_skip_while_spray_timeout_stays_blocking():
    navigating = _Harness(MissionState.NAVIGATING)
    MissionManager._tick(navigating)
    assert navigating.failures == []
    assert navigating.nav_cancels == 1
    assert navigating._nav_timeout_canceling

    spraying = _Harness(MissionState.ARM_SPRAYING)
    MissionManager._tick(spraying)
    assert spraying.failures == ['spray Action timed out']

    stale = _Harness(MissionState.VERIFYING_STOP)
    stale._stop_detector = _Detector(StopDetector.STALE)
    MissionManager._tick(stale)
    assert stale.failures == []
    assert stale.core.state == MissionState.MISSION_COMPLETED
    assert stale.core.skipped_points == 1


def test_stable_inspection_point_sends_spray_without_docking_quality_gate():
    harness = _Harness(MissionState.VERIFYING_STOP)
    harness._stop_detector = _Detector(StopDetector.STABLE)
    sent = []
    harness._send_spray_goal = lambda: sent.append(True)

    MissionManager._tick(harness)

    assert sent == [True]
    assert harness.core.state == MissionState.ARM_SPRAYING


def test_abort_and_home_returns_to_ready_only_after_resetting_then_running():
    harness = _Harness(MissionState.CANCELED)
    harness._abort_and_home_requested = True
    harness._abort_reset_sent = True

    MissionManager._on_motion_control_state(
        harness, SimpleNamespace(data='RUNNING'))
    assert harness._abort_and_home_requested

    MissionManager._on_motion_control_state(
        harness, SimpleNamespace(data='RESETTING'))
    assert harness._abort_reset_started

    MissionManager._on_motion_control_state(
        harness, SimpleNamespace(data='RUNNING'))
    assert not harness._abort_and_home_requested
    assert not harness._abort_reset_sent
    assert harness.core.last_error == 'abort_and_home: arm HOME complete and ready'


def test_abort_and_home_keeps_recovery_pending_after_home_failure():
    harness = _Harness(MissionState.CANCELED)
    harness._abort_and_home_requested = True
    harness._abort_reset_sent = True

    MissionManager._on_motion_control_state(
        harness, SimpleNamespace(data='RESET_FAILED'))

    assert harness._abort_and_home_requested
    assert harness._abort_reset_sent
    assert harness.core.last_error == 'abort_and_home: arm HOME reset failed'


def test_late_nav_goal_acceptance_stays_canceled_after_timeout():
    class PendingResult:
        def add_done_callback(self, callback):
            self.callback = callback

    class AcceptedHandle:
        accepted = True

        def __init__(self):
            self.result_future = PendingResult()

        def get_result_async(self):
            return self.result_future

    harness = _Harness(MissionState.NAVIGATING)
    harness._nav_timeout_canceling = True
    handle = AcceptedHandle()

    MissionManager._nav_goal_response(harness, _Future(handle))

    assert harness._nav_timeout_canceling
    assert harness.nav_cancels == 1
    assert harness._nav_handle is handle


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


def test_invalid_manual_frame_coordinate_or_point_count_is_rejected():
    validator = _Validator()
    for request in (
            _manual_request(frame='odom'),
            _manual_request(x=51.0),
            _manual_request(count=65)):
        try:
            MissionManager._validate_manual_request(validator, request)
        except ValueError:
            continue
        raise AssertionError('invalid mission was accepted')

def test_manual_points_preserve_selected_pose_and_validate_input():
    validator = _Validator()
    request = _manual_request()
    points, home = MissionManager._validate_manual_request(validator, request)
    assert points[0].docking_pose[:2] == (3.0, 0.5)
    assert math.isclose(points[0].docking_pose[2], 0.2)
    assert (points[0].x, points[0].y, points[0].z) == pytest.approx(
        (2.309969, 1.890632, 0.0), abs=1e-6)
    assert home == (0.0, 0.0, 0.0)


def test_manual_qt_route_accepts_twenty_three_inspection_points():
    validator = _Validator()
    validator.parameters['max_points'] = 64
    request = _manual_request(count=23)

    points, _home = MissionManager._validate_manual_request(validator, request)

    assert len(points) == 23


def test_manual_point_requires_a_nonzero_signed_arm_offset():
    validator = _Validator()
    request = _manual_request()
    request.points[0].tree_x_m = 0.0
    request.points[0].tree_y_m = 0.0
    with pytest.raises(ValueError, match='tree offset is zero'):
        MissionManager._validate_manual_request(validator, request)


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


def test_abort_and_home_accepts_failed_state_and_is_idempotent():
    harness = _Harness(MissionState.FAILED)
    harness._reply = MissionManager._reply
    harness._nav_pending = False
    harness._spray_pending = False
    harness._nav_handle = None
    harness._spray_handle = None
    commands = []
    harness._publish_motion_command = commands.append
    harness._advance_abort_and_home = MissionManager._advance_abort_and_home.__get__(
        harness, _Harness)
    response = SimpleNamespace(success=None, message='')

    MissionManager._abort_and_home(harness, None, response)

    assert response.success
    assert harness.core.state == MissionState.FAILED
    assert commands == ['stop', 'reset']

    MissionManager._abort_and_home(harness, None, response)

    assert response.success
    assert commands == ['stop', 'reset']


def test_last_point_can_require_return_home_navigation():
    harness = _Harness(MissionState.ARM_SPRAYING)
    harness._return_home_after_mission = True
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
    harness.core.point_outcomes = [MissionCore.COMPLETED]
    harness.core.completed_points = 1

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
    assert harness.core.skipped_points == 0

    harness._nav_handle = None
    harness._nav_pending = False
    MissionManager._skip_current(harness, None, response)
    assert response.success
    assert harness.core.skipped_points == 1
    assert harness.core.state == MissionState.MISSION_COMPLETED


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

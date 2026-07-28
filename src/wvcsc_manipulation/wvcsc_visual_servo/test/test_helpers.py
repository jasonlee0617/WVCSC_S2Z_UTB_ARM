import math
import threading
from types import SimpleNamespace

import pytest

from wvcsc_interfaces.action import AlignTarget
from wvcsc_visual_servo.aim_compensation import (
    AimSolution,
    plane_error_mm,
    project_nozzle_axis,
)
from wvcsc_visual_servo.servo.actuation_monitor import (
    ActuationMonitor,
    ActuationState,
)
from wvcsc_visual_servo.servo.alignment_progress import AlignmentProgress
from wvcsc_visual_servo.servo.servo_controller import (
    ServoController,
    ServoControllerConfig,
)
from wvcsc_visual_servo.servo.servo_session import (
    DirectionGuardConfig,
    ServoDecision,
    ServoDecisionKind,
    ServoFailureKind,
    ServoSession,
    TerminalReport,
)
from wvcsc_visual_servo.servo.servo_status_policy import (
    ServoStatusAction,
    ServoStatusPolicy,
)
from wvcsc_visual_servo.servo.target_tracker import TargetState, TargetTracker
from wvcsc_visual_servo.visual_servo_node import VisualServo


ARM_JOINTS = VisualServo._ARM_JOINT_NAMES


def _runtime_config(**overrides):
    values = {
        'control_rate_hz': 30.0,
        'stale_timeout_sec': 0.75,
        'invalid_target_hold_sec': 0.25,
        'min_confidence': 0.40,
        'fine_tolerance_px': 4.0,
        'control_resume_tolerance_px': 4.0,
        'stable_duration_sec': 0.50,
        'progress_window_sec': 4.0,
        'min_progress_px': 1.0,
        'max_angular_speed': 0.45,
        'max_angular_acceleration': 3.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def _controller(**overrides):
    values = {
        'control_rate_hz': 30.0,
        'kp_xy': 1.0,
        'kd_xy': 0.005,
        'd_ema_alpha': 0.65,
        'derivative_clip_xy': 2.0,
        'max_angular_speed': 0.45,
        'max_angular_acceleration': 3.0,
        'angular_u_sign': 1.0,
        'angular_v_sign': 1.0,
    }
    values.update(overrides)
    return ServoController(ServoControllerConfig(**values))


def _actuation_monitor():
    return ActuationMonitor(
        ARM_JOINTS,
        response_timeout_sec=0.75,
        min_output_rate_hz=8.0,
        min_commanded_joint_delta_rad=0.01,
        min_actual_joint_delta_rad=0.002,
    )


def _session(**config_overrides):
    controller_keys = set(ServoControllerConfig.__dataclass_fields__)
    return ServoSession(
        _runtime_config(**config_overrides),
        _controller(**{
            key: value for key, value in config_overrides.items()
            if key in controller_keys}),
        _actuation_monitor(),
        DirectionGuardConfig(False, 1.0, 20.0, 10.0, 0.60),
    )


def _target(**overrides):
    values = {
        'valid': True,
        'confidence': 0.9,
        'image_width': 1280,
        'image_height': 720,
        'center_u': 660.0,
        'center_v': 340.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_controller_controls_two_axes_and_resets():
    controller = _controller(kp_xy=0.2)
    x, y = controller.command([0.1, -0.2], 0.02)
    assert x > 0.0 and y < 0.0
    controller.reset()
    assert controller.last_command == (0.0, 0.0)


def test_controller_bounds_norm_and_slew_output():
    controller = _controller(
        kp_xy=1.0, kd_xy=0.0,
        max_angular_speed=0.08, max_angular_acceleration=0.60)
    x, y = controller.command([0.25, -0.20], 1.0 / 30.0)
    assert math.hypot(x, y) <= 0.08
    assert abs(x) <= 0.02 and abs(y) <= 0.02


def test_controller_bounds_dt_to_two_control_periods():
    controller = _controller(kp_xy=1.0, kd_xy=0.0)
    assert controller._bounded_dt(0.05) == pytest.approx(0.05)
    assert controller._bounded_dt(0.30) == pytest.approx(2.0 / 30.0)
    assert controller._bounded_dt(0.0) == pytest.approx(0.001)


def test_controller_hold_zero_preserves_pd_history_but_resets_slew_history():
    controller = _controller(kp_xy=1.0, kd_xy=0.0, max_angular_acceleration=3.0)
    controller.command([0.1, 0.0], 1.0 / 30.0)
    controller.hold_zero()
    assert controller.last_command == (0.0, 0.0)
    x, _y = controller.command([0.1, 0.0], 1.0 / 30.0)
    assert x == pytest.approx(0.1)


def test_controller_maps_camera_optical_axes_and_real_sign_override():
    controller = _controller()
    assert controller.twist_components((0.030, -0.020)) == pytest.approx(
        (0.0, 0.0, 0.020, 0.030))
    reversed_u = _controller(angular_u_sign=-1.0)
    assert reversed_u.twist_components((0.030, -0.020)) == pytest.approx(
        (0.0, 0.0, 0.020, -0.030))


def test_actuation_monitor_distinguishes_output_from_joint_motion():
    monitor = _actuation_monitor()
    monitor.reset((0.0,) * 6)
    monitor.begin_motion(10.0, [0.0] * 6)
    monitor.observe_output(SimpleNamespace(
        joint_names=list(ARM_JOINTS),
        points=[SimpleNamespace(positions=[0.01] * 6)]),
        [0.0] * 6, True, 10.1)
    monitor.observe_joint_state([0.004] * 6, True)
    state = monitor.state
    assert state.output_count == 1
    assert state.max_commanded_joint_delta_rad == pytest.approx(0.01)
    assert state.max_joint_delta_rad == pytest.approx(0.004)


def test_actuation_monitor_reports_missing_joint_trajectory_after_motion():
    monitor = _actuation_monitor()
    monitor.state = ActuationState(
        first_motion_command_monotonic=10.0,
        motion_command_active=True)
    assert 'no JointTrajectory received' in monitor.stall_reason(10.80)


def test_actuation_monitor_accepts_frequent_output_with_joint_response():
    monitor = _actuation_monitor()
    monitor.state = ActuationState(
        first_motion_command_monotonic=10.0,
        motion_command_active=True,
        output_first_monotonic=10.05,
        output_last_monotonic=10.75,
        output_count=15,
        max_commanded_joint_delta_rad=0.02,
        max_joint_delta_rad=0.004,
        motion_epoch_joint_positions=(0.0,) * 6,
        motion_epoch_max_joint_delta_rad=0.004,
    )
    assert monitor.stall_reason(10.80) is None


def test_actuation_monitor_ignores_output_silence_after_zero_hold():
    monitor = _actuation_monitor()
    monitor.state = ActuationState(
        first_motion_command_monotonic=10.0,
        output_last_monotonic=10.05,
        motion_command_active=False)
    assert monitor.stall_reason(10.83) is None


def test_new_motion_epoch_resets_watchdog_after_zero_hold():
    monitor = _actuation_monitor()
    monitor.state = ActuationState(
        first_motion_command_monotonic=10.0,
        output_last_monotonic=10.05,
        motion_command_active=False)
    monitor.begin_motion(10.83, [0.0] * 6)
    assert monitor.state.motion_command_active
    assert monitor.state.first_motion_command_monotonic == 10.83
    assert monitor.state.output_last_monotonic is None


def test_stop_burst_publishes_zero_for_a_quarter_second(monkeypatch):
    node = object.__new__(VisualServo)
    node._config = SimpleNamespace(control_rate_hz=30.0)
    node.get_parameter = lambda _name: SimpleNamespace(value=8)
    node.zero_commands = 0
    node._publish_zero = lambda: setattr(
        node, 'zero_commands', node.zero_commands + 1)
    sleeps = []
    monkeypatch.setattr(
        'wvcsc_visual_servo.visual_servo_node.time.sleep', sleeps.append)
    node._publish_zero_count()
    assert node.zero_commands == 8
    assert sleeps == [1.0 / 30.0] * 8


def test_trigger_wait_does_not_nested_spin_an_executor_owned_node():
    class Future:
        def add_done_callback(self, callback):
            callback(self)

        @staticmethod
        def result():
            return SimpleNamespace(success=True, message='servo stopped')

    class Client:
        @staticmethod
        def wait_for_service(timeout_sec):
            return timeout_sec == pytest.approx(0.1)

        @staticmethod
        def call_async(_request):
            return Future()

    node = object.__new__(VisualServo)
    assert node._call_trigger(Client(), 0.1) == (True, 'servo stopped')


def test_nozzle_axis_projection_uses_camera_tf_and_pixel_trim():
    solution = project_nozzle_axis(
        translation=(0.010, -0.020, 0.050),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        camera=(500.0, 500.0, 640.0, 360.0, 1280, 720),
        range_m=1.0, trim=(2.0, -3.0), image_margin_px=20.0)
    assert solution.u_px == pytest.approx(647.0)
    assert solution.v_px == pytest.approx(347.0)
    assert solution.intersection == pytest.approx((0.010, -0.020, 1.0))
    assert plane_error_mm(1.0, -1.0, 500.0, 500.0, 1.0) == pytest.approx(
        math.sqrt(8.0))


def test_compensated_aim_scales_to_target_message_dimensions():
    node = object.__new__(VisualServo)
    node._camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    node._aim_solution = AimSolution(
        u_px=640.0, v_px=388.0, range_m=1.0,
        intersection=(0.0, 0.056, 1.0), forward_axis=(0.0, 0.0, 1.0))
    assert node._desired_target_pixel(640, 360) == pytest.approx((320.0, 194.0))


@pytest.mark.parametrize(
    'translation,quaternion,error', [
        ((0.0, 0.0, 1.1), (0.0, 0.0, 0.0, 1.0), 'behind'),
        ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), 'does not face'),
        ((2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 'outside'),
    ])
def test_nozzle_axis_projection_rejects_unsafe_geometry(
        translation, quaternion, error):
    with pytest.raises(ValueError, match=error):
        project_nozzle_axis(
            translation, quaternion,
            (500.0, 500.0, 640.0, 360.0, 1280, 720),
            1.0, image_margin_px=20.0)


def test_target_tracker_rejects_nonfinite_and_out_of_bounds_pixels():
    tracker = TargetTracker(
        SimpleNamespace(min_confidence=0.5),
        AlignmentProgress(4.0, 0.5, 4.0, 4.0))
    assert tracker.is_valid(_target())
    assert not tracker.is_valid(_target(center_u=math.nan))
    assert not tracker.is_valid(_target(center_v=720.0))


def test_target_terminal_snapshot_preserves_last_valid_error():
    tracker = TargetTracker(
        SimpleNamespace(min_confidence=0.5),
        AlignmentProgress(4.0, 0.5, 4.0, 4.0))
    state = TargetState(
        latest={'valid': False, 'received': 11.0, 'confidence': 0.0, 'hold': False},
        last_valid_target={
            'valid': True, 'received': 10.0, 'error_u': 3.5,
            'error_v': -2.0, 'stable_frames': 2, 'hold': False},
        target_unavailable_since=10.5)
    snapshot = tracker.terminal_snapshot(state, 11.25)
    assert snapshot['error_u'] == 3.5
    assert snapshot['error_v'] == -2.0
    assert not snapshot['terminal_target_valid']
    assert snapshot['target_unavailable_sec'] == pytest.approx(0.75)


@pytest.mark.parametrize('rate_hz', [10.0, 30.0])
def test_time_based_alignment_is_independent_of_target_frame_rate(rate_hz):
    progress = AlignmentProgress(4.0, 0.5, 4.0, 4.0)
    for index in range(int(rate_hz * 0.5) + 1):
        progress.update(2.4, -2.4, index / rate_hz)
    assert progress.aligned
    assert progress.stable_duration >= 0.5


def test_alignment_progress_handles_jitter_and_excursions():
    progress = AlignmentProgress(8.0, 0.5, 4.0, 1.0, 8.0)
    progress.update(4.9, 0.0, 0.0)
    progress.update(6.3, 0.0, 0.25)
    progress.update(6.4, 0.0, 0.5)
    assert progress.aligned
    progress.update(9.0, 0.0, 0.55)
    assert not progress.aligned


def test_alignment_requires_euclidean_pixel_tolerance():
    progress = AlignmentProgress(2.0, 0.5, 4.0, 1.0)
    progress.update(1.9, 1.4, 0.0)
    progress.update(1.9, 1.4, 0.6)
    assert not progress.aligned


def test_progress_watchdog_restarts_after_real_improvement_and_reacquisition():
    progress = AlignmentProgress(4.0, 0.5, 4.0, 1.0)
    progress.update(10.0, 0.0, 0.0)
    assert progress.stalled(4.0)
    progress.restart_progress(9.5, 0.0, 10.0)
    progress.update(9.5, 0.0, 10.0)
    assert not progress.stalled(13.99)
    assert progress.stalled(14.0)


def test_moveit_servo_status_policy_is_conservative():
    policy = ServoStatusPolicy({1, 3}, {2}, {4, 5}, {6})
    assert policy.decide(0).action == ServoStatusAction.OK
    assert policy.decide(3).action == ServoStatusAction.DECELERATE
    assert policy.decide(6).action == ServoStatusAction.OK
    assert policy.decide(2).action == ServoStatusAction.RECOVERABLE_STOP
    assert policy.decide(4).action == ServoStatusAction.SAFETY_STOP
    assert policy.decide(99).action == ServoStatusAction.SAFETY_STOP


def test_session_returns_command_hold_and_stale_target_decisions():
    session = _session()
    session.reset(10.0, 20.0, [0.0] * 6)
    session.observe_target(
        _target(), 10.0, (500.0, 500.0, 640.0, 360.0, 1280, 720),
        lambda _width, _height: (640.0, 360.0))
    command = session.step(10.05, 20.05, 8.0, True, 0)
    assert command.kind == ServoDecisionKind.COMMAND
    session.observe_target(
        _target(valid=False, confidence=0.0, image_width=0, image_height=0),
        10.10, None, None)
    assert session.step(10.20, 20.20, 8.0, True, 0).kind == ServoDecisionKind.HOLD
    stale = session.step(10.86, 20.86, 8.0, True, 0)
    assert stale.kind == ServoDecisionKind.FAIL
    assert stale.failure == ServoFailureKind.TARGET_STALE


def test_session_preserves_alignment_hold_hysteresis():
    session = _session(fine_tolerance_px=1.5, control_resume_tolerance_px=3.0)
    session.reset(0.0, 0.0)
    session.state.target.latest = {
        'valid': True, 'hold': False, 'received': 0.0,
        'error_u': 1.4, 'error_v': 0.0, 'error': (0.01, 0.0),
        'confidence': 0.9, 'stable_frames': 0}
    assert session.step(0.1, 0.1, 8.0, True, 0).kind == ServoDecisionKind.HOLD
    session.state.target.latest['error_u'] = 2.2
    assert session.step(0.2, 0.2, 8.0, True, 0).kind == ServoDecisionKind.COMMAND
    assert session.state.alignment_hold_latched
    session.state.target.latest['error_u'] = 3.1
    assert session.step(0.3, 0.3, 8.0, True, 0).kind == ServoDecisionKind.COMMAND
    assert not session.state.alignment_hold_latched


def test_session_stops_for_direction_divergence_and_requested_safety_stop():
    session = ServoSession(
        _runtime_config(), _controller(), _actuation_monitor(),
        DirectionGuardConfig(True, 1.0, 20.0, 10.0, 0.60))
    session.reset(10.0, 10.0)
    session.state.target.latest = {
        'valid': True, 'hold': False, 'received': 10.0,
        'error_u': -60.0, 'error_v': -86.0, 'error': (-0.12, -0.17),
        'confidence': 0.9, 'stable_frames': 0}
    assert session.step(10.0, 10.0, 8.0, True, 0).kind == ServoDecisionKind.COMMAND
    session.state.target.latest['error_u'] = -74.0
    session.state.target.latest['received'] = 11.0
    failed = session.step(11.0, 11.0, 8.0, True, 0)
    assert failed.failure == ServoFailureKind.SERVO_DIRECTION_DIVERGENCE
    session.request_stop(ServoFailureKind.SERVO_SAFETY_STOP, 'collision')
    assert session.step(11.1, 11.1, 8.0, True, 0).failure == ServoFailureKind.SERVO_SAFETY_STOP


def test_session_reports_actuation_stall_and_terminal_diagnostics():
    session = _session()
    session.reset(10.0, 10.0, [0.0] * 6)
    session.state.target.latest = {
        'valid': True, 'hold': False, 'received': 10.0,
        'error_u': 20.0, 'error_v': 0.0, 'error': (0.04, 0.0),
        'confidence': 0.9, 'stable_frames': 0}
    assert session.step(10.0, 10.0, 8.0, True, 0).kind == ServoDecisionKind.COMMAND
    session.state.target.latest['received'] = 10.80
    failed = session.step(10.80, 10.80, 8.0, True, 0)
    assert failed.failure == ServoFailureKind.SERVO_ACTUATION_STALL
    report = session.terminal_report(
        failed.failure, failed.message, 10.80, 0, True)
    assert 'servo_outputs=0' in report.summary
    assert 'servo_status=0' in report.summary


def _node_session_harness():
    node = object.__new__(VisualServo)
    node._lock = threading.Lock()
    node._busy = True
    node._active_mission = 'mission-1'
    node._active_target = 'target-1'
    node._aim_solution = object()
    node._camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    node._session = _session()
    node._session.reset(10.0, 10.0)
    node._now = lambda: 10.1
    node.zero_commands = 0
    node._publish_zero = lambda: setattr(
        node, 'zero_commands', node.zero_commands + 1)
    return node


def test_node_target_callback_holds_matching_invalid_target_without_lock_reentry():
    node = _node_session_harness()
    node._session.state.target.latest = {
        'valid': True, 'received': 10.0, 'error': (0.0, 0.0),
        'stable_frames': 1, 'confidence': 0.9}
    callback = threading.Thread(
        target=node._on_target,
        args=(_target(
            mission_id='mission-1', target_id='target-1',
            valid=False, confidence=0.0, image_width=0, image_height=0),))
    callback.start()
    callback.join(timeout=0.25)
    assert not callback.is_alive()
    assert node._session.latest['hold']
    assert node.zero_commands == 1


def test_node_target_callback_ignores_non_matching_target():
    node = _node_session_harness()
    node._session.state.target.latest = {'valid': True, 'received': 10.0}
    node._on_target(_target(
        mission_id='mission-1', target_id='other',
        valid=False, confidence=0.0, image_width=0, image_height=0))
    assert node._session.latest == {'valid': True, 'received': 10.0}


def test_node_servo_status_maps_recoverable_and_hard_stops_to_session():
    for status, expected in (
            (2, ServoFailureKind.SERVO_SINGULARITY),
            (4, ServoFailureKind.SERVO_SAFETY_STOP),
            (5, ServoFailureKind.SERVO_SAFETY_STOP),
            (99, ServoFailureKind.SERVO_SAFETY_STOP)):
        node = _node_session_harness()
        node._policy = ServoStatusPolicy({1, 3}, {2}, {4, 5}, {6})
        node._servo_status = 0
        node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
        node._on_servo_status(SimpleNamespace(data=status))
        assert node._session.state.stop_failure == expected
        assert node.zero_commands == 1


def test_node_joint_callback_ignores_vehicle_only_joint_state():
    node = _node_session_harness()
    node._on_joint_state(SimpleNamespace(name=['left_front_joint'], position=[0.25]))
    assert node._session._joint_positions == []
    node._on_joint_state(SimpleNamespace(
        name=[f'joint{index}' for index in range(6, 0, -1)],
        position=[float(index) for index in range(6, 0, -1)]))
    assert node._session._joint_positions == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_servo_lifecycle_starts_once_then_reuses_zero_braked_loop():
    node = object.__new__(VisualServo)
    node._servo_lifecycle = 'never_started'
    node._start_client = 'start'
    node._publish_zero_count = lambda: None
    node.get_parameter = lambda _name: SimpleNamespace(value=12.0)
    calls = []
    node._call_trigger = lambda client, timeout: (
        calls.append((client, timeout)) or (True, 'ok'))
    assert node._activate_servo() == (True, 'ok')
    assert node._brake_servo('first_target_done') == (True, 'zero commands published')
    assert node._activate_servo() == (True, 'already running')
    assert calls == [('start', 12.0)]


@pytest.mark.parametrize('outcome', ['aligned', 'failed', 'canceled'])
def test_each_execute_exit_path_calls_servo_stop_once(outcome, monkeypatch):
    monkeypatch.setattr(
        'wvcsc_visual_servo.visual_servo_node.rclpy.ok', lambda: True)

    class Logger:
        def __init__(self):
            self.lines = []

        def info(self, message):
            self.lines.append(message)

        warn = error = info

    class Goal:
        request = SimpleNamespace(
            timeout=8.0, mission_id='mission-1',
            target_id='target-1', working_range_m=1.0,
            desired_u_px=640.0, desired_v_px=360.0,
            image_width=1280, image_height=720)
        is_cancel_requested = outcome == 'canceled'

        def __init__(self):
            self.terminal = ''

        def succeed(self):
            self.terminal = 'succeeded'

        def abort(self):
            self.terminal = 'aborted'

        def canceled(self):
            self.terminal = 'canceled'

        def publish_feedback(self, _feedback):
            pass

    latest = {
        'valid': True, 'hold': False, 'received': 0.0,
        'error_u': 1.0, 'error_v': -1.0, 'error': (1.0, -1.0),
        'stable_frames': 10, 'confidence': 0.9}

    class Session:
        def reset(self, *_args):
            pass

        @staticmethod
        def terminal_snapshot(_now, supplied=None):
            return dict(latest if supplied is None else supplied)

        @staticmethod
        def terminal_report(_failure, _message, _now, _status, _camera, snapshot):
            return TerminalReport(snapshot, '[VISUAL_SERVO] test_failure')

        @staticmethod
        def hold_zero():
            pass

        @staticmethod
        def twist_components(_command):
            return 0.0, 0.0, 0.0, 0.0

        @staticmethod
        def step(_now, _monotonic, _timeout, _camera_ready, _status):
            if outcome == 'aligned':
                return ServoDecision(
                    ServoDecisionKind.SUCCESS, latest=latest,
                    stable_duration_sec=0.5)
            return ServoDecision(
                ServoDecisionKind.FAIL, latest=latest,
                failure=ServoFailureKind.TIMEOUT,
                message='visual alignment timed out')

    node = object.__new__(VisualServo)
    node._config = SimpleNamespace(default_timeout_sec=8.0, control_rate_hz=30.0)
    node._lock = threading.Lock()
    node._camera = (500.0, 500.0, 640.0, 360.0, 1280, 720)
    node._session = Session()
    node._servo_status = 0
    node._angular_u_sign = 1.0
    node._angular_v_sign = 1.0
    node._publish_zero = lambda: None
    node._publish_zero_count = lambda: None
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    node._now = lambda: 0.0
    node._aim_solution = None
    logger = Logger()
    node.get_logger = lambda: logger
    calls = []
    node._activate_servo = lambda: (calls.append('start') or (True, ''))
    node._brake_servo = lambda _reason: (calls.append('brake') or (True, ''))
    node._prepare_aim_compensation = lambda _range: (
        setattr(node, '_aim_solution',
                SimpleNamespace(u_px=640.0, v_px=360.0, range_m=1.0))
        or (True, ''))
    goal = Goal()
    result = node._execute(goal)
    assert calls == ['start', 'brake']
    assert goal.terminal == {
        'aligned': 'succeeded', 'failed': 'aborted', 'canceled': 'canceled'}[outcome]
    if outcome == 'failed':
        assert result.error_code == AlignTarget.Result.TIMEOUT
    assert any('[VISUAL_SERVO] 进入伺服 target=target-1' in line
               for line in logger.lines)

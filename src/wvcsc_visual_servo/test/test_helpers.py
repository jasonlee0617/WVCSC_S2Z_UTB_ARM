import json
import math
import threading
from types import SimpleNamespace

import pytest

from wvcsc_visual_servo.servo.pid_controller import (
    PIDController2D,
    ServoControlConfig,
)
from wvcsc_visual_servo.servo.alignment_progress import AlignmentProgress
from wvcsc_visual_servo.servo.command_limiter import (
    bounded_control_dt,
    limit_xy_norm,
    slew,
)
from wvcsc_visual_servo.servo.debug_snapshot import (
    DEBUG_DEFAULTS,
    debug_json,
    debug_publish_due,
)
from wvcsc_visual_servo.servo.servo_status_policy import (
    ServoStatusAction,
    ServoStatusPolicy,
)
from wvcsc_visual_servo.servo.target_estimator import SimpleTargetPredictor2D
from wvcsc_visual_servo.visual_servo_node import VisualServo
from wvcsc_interfaces.action import AlignTarget


def test_pid_controls_two_image_axes_and_resets():
    controller = PIDController2D(ServoControlConfig(kp_xy=0.2))
    x, y, _debug = controller.step([0.1, -0.2], 0.02)
    assert x > 0.0 and y < 0.0
    controller.reset()


def test_limiter_caps_norm_and_acceleration():
    x, y = limit_xy_norm(3.0, 4.0, 1.0)
    assert math.isclose(math.hypot(x, y), 1.0)
    assert math.isclose(slew(1.0, 0.0, 2.0, 0.1), 0.2)


def test_joint_debug_ignores_vehicle_only_joint_state():
    node = object.__new__(VisualServo)
    node._lock = threading.Lock()
    node._joint_positions = [1.0] * 6

    node._on_joint_state(SimpleNamespace(
        name=['left_front_joint'], position=[0.25]))
    assert node._joint_positions == [1.0] * 6

    node._on_joint_state(SimpleNamespace(
        name=[f'joint{index}' for index in range(6, 0, -1)],
        position=[float(index) for index in range(6, 0, -1)]))
    assert node._joint_positions == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_stop_burst_publishes_zero_for_a_quarter_second(monkeypatch):
    node = object.__new__(VisualServo)
    node._config = SimpleNamespace(control_rate_hz=20.0)
    node.get_parameter = lambda _name: SimpleNamespace(value=5)
    node.zero_commands = 0
    node._publish_zero = lambda: setattr(
        node, 'zero_commands', node.zero_commands + 1)
    sleeps = []
    monkeypatch.setattr(
        'wvcsc_visual_servo.visual_servo_node.time.sleep',
        sleeps.append)

    node._publish_zero_count()

    assert node.zero_commands == 5
    assert sleeps == [0.05] * 5
    assert math.isclose(sum(sleeps), 0.25)


def test_control_dt_uses_actual_20hz_period_with_bounded_overrun():
    assert math.isclose(bounded_control_dt(0.08, 20.0), 0.08)
    assert math.isclose(bounded_control_dt(0.30, 20.0), 0.10)
    assert math.isclose(bounded_control_dt(0.0, 20.0), 0.001)


def test_pid_accepts_node_bounded_dt_without_hidden_50ms_cap():
    controller = PIDController2D(ServoControlConfig(kp_xy=0.0, ki_xy=1.0))
    _x, _y, debug = controller.step([1.0, 0.0], 0.08)
    assert math.isclose(debug['integral'][0], 0.08)


def test_compensated_pid_gain_remains_bounded_by_motion_limits():
    controller = PIDController2D(ServoControlConfig(kp_xy=1.0, kd_xy=0.005))
    x, y, _debug = controller.step([0.25, -0.20], 0.05)
    x, y = limit_xy_norm(x, y, 0.08)
    x = slew(x, 0.0, 0.60, 0.05)
    y = slew(y, 0.0, 0.60, 0.05)
    assert math.hypot(x, y) <= 0.08
    assert abs(x) <= 0.03 and abs(y) <= 0.03


def test_time_based_alignment_is_independent_of_target_frame_rate():
    for rate_hz in (10.0, 30.0):
        progress = AlignmentProgress(4.0, 0.5, 4.0, 4.0)
        for index in range(int(rate_hz * 0.5) + 1):
            progress.update(4.0, -3.0, index / rate_hz)
        assert progress.aligned
        assert progress.stable_duration >= 0.5


def test_alignment_excursion_resets_stable_duration():
    progress = AlignmentProgress(4.0, 0.5, 4.0, 4.0)
    progress.update(3.0, 3.0, 0.0)
    progress.update(3.0, 3.0, 0.4)
    progress.update(9.0, 3.0, 0.45)
    progress.update(3.0, 3.0, 0.8)
    assert not progress.aligned
    assert progress.stable_duration == 0.0


def test_progress_watchdog_stalls_but_resets_after_real_improvement():
    progress = AlignmentProgress(4.0, 0.5, 4.0, 4.0)
    progress.update(100.0, 0.0, 0.0)
    progress.update(97.0, 0.0, 3.9)
    assert not progress.stalled(3.9)
    assert progress.stalled(4.0)

    progress.update(95.0, 0.0, 4.0)
    assert not progress.stalled(7.9)
    progress.update(90.0, 0.0, 7.9)
    assert not progress.stalled(11.8)


def test_subpixel_steps_accumulate_into_meaningful_progress():
    progress = AlignmentProgress(4.0, 0.5, 4.0, 1.0)
    progress.update(10.0, 0.0, 0.0)
    progress.update(9.6, 0.0, 1.0)
    progress.update(9.2, 0.0, 2.0)
    progress.update(8.8, 0.0, 3.0)
    assert not progress.stalled(4.0)


def test_progress_window_restarts_after_target_reacquisition():
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


def _status_harness(busy):
    node = object.__new__(VisualServo)
    node._policy = ServoStatusPolicy({1, 3}, {2}, {4, 5}, {6})
    node._lock = threading.Lock()
    node._busy = busy
    node._servo_status = 0
    node._stop_code = None
    node._stop_message = ''
    node.zero_commands = 0
    node._publish_zero = lambda: setattr(
        node, 'zero_commands', node.zero_commands + 1)
    node._publish_debug = lambda *_args, **_kwargs: None
    return node


def test_singularity_status_is_recoverable_without_global_motion_stop():
    node = _status_harness(True)
    node._on_servo_status(SimpleNamespace(data=2))
    assert node._stop_code == AlignTarget.Result.SERVO_SINGULARITY
    assert node.zero_commands == 1
    assert not hasattr(node, '_motion_command')


def test_collision_joint_bound_and_unknown_status_are_hard_safety_stops():
    for status in (4, 5, 99):
        node = _status_harness(True)
        node._on_servo_status(SimpleNamespace(data=status))
        assert node._stop_code == AlignTarget.Result.SERVO_SAFETY_STOP
        assert node.zero_commands == 1


def test_idle_servo_status_does_not_stop_or_lock_motion():
    node = _status_harness(False)
    node._on_servo_status(SimpleNamespace(data=2))
    assert node._stop_code is None
    assert node.zero_commands == 0


def test_invalid_matching_target_briefly_holds_zero_without_erasing_lock():
    class Predictor:
        reset_calls = 0

        def reset(self):
            self.reset_calls += 1

    node = object.__new__(VisualServo)
    node._lock = threading.Lock()
    node._busy = True
    node._active_mission = 'mission-1'
    node._active_tree = 'tree-1'
    node._active_target = 'fruit-1'
    node._config = SimpleNamespace(
        min_confidence=0.40,
        invalid_target_hold_sec=0.25,
        desired_offset_u_px=0.0,
        desired_offset_v_px=0.0,
        fine_tolerance_px=4.0,
    )
    node._latest = {
        'valid': True,
        'received': 10.0,
        'error': (0.0, 0.0),
        'stable_frames': 6,
        'confidence': 0.9,
    }
    node._last_valid_target = dict(node._latest)
    node._target_unavailable_since = None
    node._stable_frames = 6
    node._progress = AlignmentProgress(4.0, 0.5, 4.0, 4.0)
    node._predictor = Predictor()
    node._now = lambda: 10.1
    node.zero_commands = 0
    node._publish_zero = lambda: setattr(
        node, 'zero_commands', node.zero_commands + 1)

    node._on_target(SimpleNamespace(
        mission_id='mission-1', tree_id='tree-1', target_id='fruit-1',
        valid=False, confidence=0.0, image_width=0, image_height=0,
    ))

    assert node._latest['valid']
    assert node._latest['hold']
    assert node._latest['stable_frames'] == 0
    assert node._predictor.reset_calls == 1
    assert node._target_unavailable_since == 10.1
    assert node.zero_commands == 1


def test_terminal_snapshot_preserves_last_valid_error_during_target_loss():
    node = object.__new__(VisualServo)
    node._lock = threading.Lock()
    node._latest = {
        'valid': False, 'received': 11.0, 'confidence': 0.0, 'hold': False}
    node._last_valid_target = {
        'valid': True, 'received': 10.0, 'error_u': 3.5, 'error_v': -2.0,
        'stable_frames': 2, 'hold': False}
    node._target_unavailable_since = 10.5

    snapshot = node._terminal_target_snapshot(11.25)

    assert snapshot['error_u'] == 3.5
    assert snapshot['error_v'] == -2.0
    assert not snapshot['terminal_target_valid']
    assert math.isclose(snapshot['target_unavailable_sec'], 0.75)


def test_non_matching_target_does_not_invalidate_the_active_target():
    node = object.__new__(VisualServo)
    node._lock = threading.Lock()
    node._busy = True
    node._active_mission = 'mission-1'
    node._active_tree = 'tree-1'
    node._active_target = 'fruit-1'
    node._latest = {'valid': True, 'received': 10.0}
    node._on_target(SimpleNamespace(
        mission_id='mission-1', tree_id='tree-1', target_id='fruit-other',
        valid=False, confidence=0.0, image_width=0, image_height=0,
    ))

    assert node._latest == {'valid': True, 'received': 10.0}


def test_visual_servo_debug_json_has_stable_complete_schema():
    payload = json.loads(debug_json(event='control', error_u_px=12.5))
    assert list(payload) == list(DEBUG_DEFAULTS)
    assert payload['event'] == 'control'
    assert payload['error_u_px'] == 12.5


def test_visual_servo_debug_rate_is_limited_unless_forced():
    assert debug_publish_due(10.0, None, 5.0)
    assert not debug_publish_due(10.1, 10.0, 5.0)
    assert debug_publish_due(10.21, 10.0, 5.0)
    assert debug_publish_due(10.01, 10.0, 5.0, force=True)


def test_predictor_uses_bounded_horizon():
    predictor = SimpleTargetPredictor2D()
    predictor.update([1.0, 2.0], [2.0, -1.0], 10.0)
    position, velocity = predictor.predict_to(11.0, 0.1)
    assert position == pytest.approx((1.2, 1.9))
    assert velocity == pytest.approx((2.0, -1.0))


@pytest.mark.parametrize('outcome', ['aligned', 'timeout', 'canceled', 'stale'])
def test_each_execute_exit_path_calls_servo_stop_once(outcome, monkeypatch):
    monkeypatch.setattr(
        'wvcsc_visual_servo.visual_servo_node.rclpy.ok', lambda: True)

    class Resettable:
        def reset(self):
            pass

    class Progress(Resettable):
        aligned = outcome == 'aligned'
        stable_duration = 0.5

        @staticmethod
        def stalled(_now):
            return False

    class Logger:
        info = warn = error = staticmethod(lambda *_args: None)

    class Goal:
        request = SimpleNamespace(
            timeout=8.0, mission_id='mission-1',
            tree_id='tree-1', target_id='fruit-1')
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

    node = object.__new__(VisualServo)
    node._config = SimpleNamespace(
        default_timeout_sec=8.0, control_rate_hz=20.0,
        stale_timeout_sec=0.75)
    node._lock = threading.Lock()
    node._camera = object()
    node._busy = True
    node._controller = Resettable()
    node._predictor = Resettable()
    node._progress = Progress()
    node._start_client = 'start'
    node._stop_client = 'stop'
    node._publish_debug = lambda *_args, **_kwargs: None
    node._publish_zero = lambda: None
    node._publish_zero_count = lambda: None
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    node.get_logger = lambda: Logger()
    timeout_times = iter(
        [0.0, 0.8, 0.8, 0.8, 0.8]
        if outcome == 'stale' else [0.0, 9.0, 9.0])
    node._now = (
        (lambda: next(timeout_times, 9.0))
        if outcome in {'timeout', 'stale'} else (lambda: 0.0))
    calls = []

    def trigger(client):
        calls.append(client)
        if client == 'start':
            valid_target = {
                'valid': True, 'hold': False, 'received': node._now(),
                'error_u': 1.0, 'error_v': -1.0,
                'error': (1.0, -1.0),
                'stable_frames': 10, 'confidence': 0.9,
            }
            node._last_valid_target = dict(valid_target)
            node._latest = (
                {'valid': False, 'hold': False, 'received': 0.0,
                 'confidence': 0.0}
                if outcome == 'stale' else valid_target)
        return True, ''

    node._call_trigger = trigger
    goal = Goal()

    result = node._execute(goal)

    assert calls == ['start', 'stop']
    assert goal.terminal == {
        'aligned': 'succeeded',
        'timeout': 'aborted',
        'canceled': 'canceled',
        'stale': 'aborted',
    }[outcome]
    if outcome == 'stale':
        assert result.error_code == AlignTarget.Result.TARGET_STALE
        assert result.final_error_u == 1.0
        assert result.final_error_v == -1.0

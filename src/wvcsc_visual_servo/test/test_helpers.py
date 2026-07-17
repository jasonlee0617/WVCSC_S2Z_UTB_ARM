import json
import math
import threading
from types import SimpleNamespace

import numpy as np

from wvcsc_visual_servo.controllers.pid_controller import (
    PIDController3D,
    ServoControlConfig,
)
from wvcsc_visual_servo.servo.command_limiter import limit_xy_norm, slew
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


def test_pid_disables_depth_axis_and_resets():
    controller = PIDController3D(ServoControlConfig(kp_xy=0.2))
    x, y, z, _debug = controller.step([0.1, -0.2, 1.0], 0.02)
    assert x > 0.0 and y < 0.0 and z == 0.0
    controller.reset()


def test_limiter_caps_norm_and_acceleration():
    x, y = limit_xy_norm(3.0, 4.0, 1.0)
    assert math.isclose(math.hypot(x, y), 1.0)
    assert math.isclose(slew(1.0, 0.0, 2.0, 0.1), 0.2)


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
        fine_tolerance_px=8.0,
    )
    node._latest = {
        'valid': True,
        'received': 10.0,
        'error': np.array([0.0, 0.0]),
        'stable_frames': 6,
        'confidence': 0.9,
    }
    node._stable_frames = 6
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
    assert node._predictor.reset_calls == 0
    assert node.zero_commands == 1


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
    assert np.allclose(position, [1.2, 1.9])
    assert np.allclose(velocity, [2.0, -1.0])

import threading

import pytest
from wvcsc_interfaces.action import ExecuteSpray

from wvcsc_arm_task.motion_state import MotionControlState
from wvcsc_arm_task.spray_task import SprayTask


class _Arm:
    def __init__(self, outcomes):
        self.outcomes = iter(outcomes)
        self.calls = []

    def move_joints(self, positions):
        self.calls.append(positions)
        return next(self.outcomes)

    def cancel(self):
        self.calls.append('cancel')


class _Sequence:
    _run_sequence = SprayTask._run_sequence
    _move = SprayTask._move
    _aborted = SprayTask._aborted
    _run_timer_spray = SprayTask._run_timer_spray
    _validate_goal = SprayTask._validate_goal
    _claim = SprayTask._claim
    _release = SprayTask._release

    def __init__(self, outcomes):
        self._observe_left = ['left']
        self._observe_right = ['right']
        self._home = ['home']
        self._abort = threading.Event()
        self._busy_mutex = threading.Lock()
        self._busy = False
        self._min_duration = 0.2
        self._max_duration = 10.0
        self._use_vision_alignment = False
        self._use_spray_action = False
        self.state = MotionControlState()
        self.arm = _Arm(outcomes)


def _run(sequence, side='left', canceled=lambda: False):
    feedback = []
    result = sequence._run_sequence(
        side,
        0.0,
        cancel_requested=canceled,
        feedback=lambda phase, progress, text: feedback.append(
            (phase, progress, text)),
    )
    return result, feedback


@pytest.mark.parametrize('side,observe', [
    ('left', ['left']),
    ('right', ['right']),
])
def test_success_requires_observe_and_home_motion(side, observe):
    sequence = _Sequence([True, True])
    (code, _message), feedback = _run(sequence, side=side)
    assert code == ExecuteSpray.Result.OK
    assert sequence.arm.calls == [observe, ['home']]
    assert feedback[-1][0] == ExecuteSpray.Feedback.COMPLETED


def test_observe_failure_returns_home_but_does_not_report_success():
    sequence = _Sequence([False, True])
    (code, _message), _feedback = _run(sequence)
    assert code == ExecuteSpray.Result.OBSERVE_FAILED
    assert sequence.arm.calls == [['left'], ['home']]


def test_home_failure_is_reported():
    sequence = _Sequence([True, False])
    (code, _message), _feedback = _run(sequence, side='right')
    assert code == ExecuteSpray.Result.HOME_FAILED
    assert sequence.arm.calls == [['right'], ['home']]


def test_cancel_stops_before_home_and_never_reports_success():
    sequence = _Sequence([True])
    (code, _message), _feedback = _run(sequence, canceled=lambda: True)
    assert code == ExecuteSpray.Result.CANCELED
    assert sequence.arm.calls == [['left']]


@pytest.mark.parametrize('side,duration', [
    ('ahead', 2.0),
    ('left', 0.1),
    ('right', 10.1),
    ('left', float('nan')),
])
def test_invalid_goal_fields_are_rejected(side, duration):
    sequence = _Sequence([])
    assert sequence._validate_goal('mission', 'tree', side, duration)


def test_busy_and_locked_sequences_cannot_claim_a_goal():
    sequence = _Sequence([])
    assert sequence._claim()
    assert not sequence._claim()
    sequence._release()
    sequence.state.stop()
    assert not sequence._claim()


def test_optional_vision_and_spray_failures_return_home():
    vision = _Sequence([True, True])
    vision._use_vision_alignment = True
    vision._align_target = lambda *_args: (False, False, 'target stale')
    (code, _message), _feedback = _run(vision)
    assert code == ExecuteSpray.Result.VISION_FAILED
    assert vision.arm.calls == [['left'], ['home']]

    spray = _Sequence([True, True])
    spray._use_spray_action = True
    spray._spray_target = lambda *_args: (False, False, 'valve failed')
    (code, _message), _feedback = _run(spray)
    assert code == ExecuteSpray.Result.SPRAY_FAILED
    assert spray.arm.calls == [['left'], ['home']]


def test_servo_safety_failure_locks_motion_and_does_not_become_skip():
    sequence = _Sequence([True])
    sequence._use_vision_alignment = True
    sequence._align_target = lambda *_args: (
        False, False, '[SAFETY] collision halt')

    (code, _message), _feedback = _run(sequence)

    assert code == ExecuteSpray.Result.INTERNAL_ERROR
    assert sequence.state.locked
    assert sequence.arm.calls == [['left'], 'cancel']


def test_optional_vision_and_spray_success_preserve_home_requirement():
    sequence = _Sequence([True, True])
    sequence._use_vision_alignment = True
    sequence._use_spray_action = True
    sequence._align_target = lambda *_args: (True, False, 'aligned')
    sequence._spray_target = lambda *_args: (True, False, 'sprayed')

    (code, _message), feedback = _run(sequence)

    assert code == ExecuteSpray.Result.OK
    assert sequence.arm.calls == [['left'], ['home']]
    assert [item[0] for item in feedback] == [
        ExecuteSpray.Feedback.MOVING_TO_OBSERVE,
        ExecuteSpray.Feedback.ALIGNING,
        ExecuteSpray.Feedback.SPRAYING,
        ExecuteSpray.Feedback.RETURNING_HOME,
        ExecuteSpray.Feedback.COMPLETED,
    ]

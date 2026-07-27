from wvcsc_arm_task.motion.motion_state import (
    MotionControlState,
    begin_reset,
    perform_reset,
)

from wvcsc_arm_task.motion.motion_control_keyboard import command_for_key


def test_motion_control_keyboard_mapping():
    assert command_for_key(' ') == 'stop'
    assert command_for_key('h') == 'reset'
    assert command_for_key('r') == 'resume'
    assert command_for_key('x') is None
    assert command_for_key('?') is None


class _Arm:
    def __init__(
            self, stop_success=True, open_success=True, home_success=True):
        self.calls = []
        self.stop_success = stop_success
        self.open_success = open_success
        self.home_success = home_success

    def cancel_and_wait(self):
        self.calls.append('cancel_and_wait')
        return self.stop_success

    def control_gripper(self, open_gripper, allow_locked):
        assert open_gripper
        assert allow_locked
        self.calls.append('open')
        return self.open_success

    def move_joints(self, positions, allow_locked):
        assert allow_locked
        self.calls.append(('home', positions))
        return self.home_success


def test_stop_blocks_until_resume():
    state = MotionControlState()
    assert not state.locked
    state.stop()
    assert state.locked
    assert state.resume()
    assert not state.locked


def test_reset_is_single_flight_and_stays_locked():
    state = MotionControlState()
    assert state.begin_reset()
    assert state.locked
    assert state.reset_in_progress
    assert not state.begin_reset()
    assert not state.resume()
    state.finish_reset()
    assert state.locked
    assert state.resume()


def test_reset_stops_opens_and_moves_to_zero_home_in_order():
    state = MotionControlState()
    arm = _Arm()
    home = [0.0] * 6
    assert begin_reset(state, arm)
    assert perform_reset(state, arm, home)
    assert arm.calls == ['cancel_and_wait', 'open', ('home', home)]
    assert state.locked
    assert not state.reset_in_progress


def test_reset_failure_skips_home_and_remains_locked():
    state = MotionControlState()
    arm = _Arm(open_success=False)
    assert begin_reset(state, arm)
    assert not perform_reset(state, arm, [0.0] * 6)
    assert arm.calls == ['cancel_and_wait', 'open']
    assert state.locked
    assert not state.reset_in_progress


def test_reset_home_failure_remains_locked():
    state = MotionControlState()
    arm = _Arm(home_success=False)
    home = [0.0] * 6
    assert begin_reset(state, arm)
    assert not perform_reset(state, arm, home)
    assert arm.calls == ['cancel_and_wait', 'open', ('home', home)]
    assert state.locked
    assert not state.reset_in_progress


def test_reset_does_not_open_gripper_until_all_motion_is_stopped():
    state = MotionControlState()
    arm = _Arm(stop_success=False)
    assert not begin_reset(state, arm)
    assert arm.calls == ['cancel_and_wait']
    assert state.locked
    assert not state.reset_in_progress


def test_reset_abort_predicate_prevents_home_motion():
    state = MotionControlState()
    arm = _Arm()
    assert begin_reset(state, arm)

    assert not perform_reset(
        state, arm, [0.0] * 6, abort_requested=lambda: True)
    assert arm.calls == ['cancel_and_wait']
    assert state.locked
    assert not state.reset_in_progress

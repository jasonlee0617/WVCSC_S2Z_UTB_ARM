from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from moveit_msgs.msg import MoveItErrorCodes
from pymoveit2 import MoveIt2State

from wvcsc_arm_task.alicia_moveit import AliciaMoveIt


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class _Logger:
    def error(self, _message):
        pass


class _Node:
    def __init__(self):
        self.publisher = _Publisher()

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher

    def get_logger(self):
        return _Logger()


class _MoveIt:
    end_effector_name = 'tool0'

    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.plan_calls = []
        self.executed = []
        self._states = []
        self._error = None
        self.start_execution = True

    def plan_async(self, **kwargs):
        self.plan_calls.append(kwargs)
        return _Future(object())

    def get_trajectory(self, *_args, **_kwargs):
        return self.trajectory

    def execute(self, trajectory):
        self.executed.append(trajectory)
        if self.start_execution:
            self._states = [MoveIt2State.REQUESTING, MoveIt2State.IDLE]
            self._error = SimpleNamespace(val=MoveItErrorCodes.SUCCESS)

    def query_state(self):
        return self._states.pop(0) if self._states else MoveIt2State.IDLE

    def get_last_execution_error_code(self):
        return self._error


class _RetimeClient:
    def __init__(self, response):
        self.response = response
        self.calls = 0

    def wait_for_service(self, timeout_sec):
        return timeout_sec > 0.0

    def call_async(self, _request):
        self.calls += 1
        return _Future(self.response)


class _Gripper:
    def __init__(self):
        self.cancelled = False

    def command(self, *_args):
        return True

    def cancel(self):
        self.cancelled = True

    def wait_idle(self, _timeout):
        return True


class _Activity:
    def __init__(self, idle=True):
        self.idle = idle
        self.waits = 0

    def wait_idle(self, _timeout):
        self.waits += 1
        return self.idle


def _trajectory(times=(0, 100_000_000)):
    return SimpleNamespace(
        joint_names=list(AliciaMoveIt.JOINT_NAMES),
        points=[SimpleNamespace(
            positions=[0.0] * 6,
            time_from_start=SimpleNamespace(
                sec=t // 1_000_000_000, nanosec=t % 1_000_000_000),
        ) for t in times],
    )


def _adapter(retimed):
    node = _Node()
    planned = _trajectory()
    moveit = _MoveIt(planned)
    retime = _RetimeClient(SimpleNamespace(success=True, retimed=retimed))
    request_factory = lambda: SimpleNamespace(
        trajectory=None, group_name='', velocity_scaling=0.0,
        acceleration_scaling=0.0)
    adapter = AliciaMoveIt(
        node, moveit=moveit, retime_client=retime, gripper=_Gripper(),
        retime_request_factory=request_factory, execution_timeout=0.2,
        arm_activity=_Activity(), gripper_activity=_Activity())
    return adapter, moveit, retime


def test_joint_motion_does_not_call_retime_service():
    adapter, moveit, retime = _adapter(_trajectory())
    assert adapter.move_joints(AliciaMoveIt.HOME)
    assert retime.calls == 0
    assert len(moveit.executed) == 1


def test_cartesian_motion_retimes_exactly_once_before_execute():
    retimed = _trajectory((0, 200_000_000))
    adapter, moveit, retime = _adapter(retimed)
    assert adapter.move_cartesian([0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
    assert retime.calls == 1
    assert moveit.executed == [retimed]


def test_invalid_retimed_trajectory_is_not_executed():
    adapter, moveit, retime = _adapter(_trajectory((10, 10)))
    assert not adapter.move_cartesian(
        [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
    assert retime.calls == 1
    assert moveit.executed == []


def test_retime_failure_response_is_not_executed():
    adapter, moveit, retime = _adapter(_trajectory())
    retime.response = SimpleNamespace(success=False, retimed=_trajectory())
    assert not adapter.move_cartesian(
        [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])
    assert retime.calls == 1
    assert moveit.executed == []


def test_cancel_locks_out_inflight_operation_epoch():
    adapter, moveit, _retime = _adapter(_trajectory())
    adapter.state.stop()
    adapter.cancel()
    assert not adapter.move_joints(AliciaMoveIt.HOME)
    assert moveit.executed == []


def test_stale_success_is_ignored_until_new_execution_really_starts():
    adapter, moveit, _retime = _adapter(_trajectory())
    moveit._error = SimpleNamespace(val=MoveItErrorCodes.SUCCESS)
    moveit.start_execution = False
    assert not adapter.move_joints(AliciaMoveIt.HOME)
    assert len(moveit.executed) == 1


def test_cancel_and_wait_checks_arm_and_gripper_are_idle():
    adapter, moveit, _retime = _adapter(_trajectory())
    moveit._states = [MoveIt2State.EXECUTING, MoveIt2State.IDLE]
    assert adapter.cancel_and_wait(timeout=0.2)
    assert adapter._gripper.cancelled
    assert adapter._node.publisher.messages == ['stop']
    assert adapter._arm_activity.waits == 1
    assert adapter._gripper_activity.waits == 1

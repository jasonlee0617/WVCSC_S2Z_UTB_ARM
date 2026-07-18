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
    def __init__(self):
        self.infos = []

    def error(self, _message):
        pass

    def info(self, message):
        self.infos.append(message)


class _Node:
    def __init__(self):
        self.publisher = _Publisher()
        self.logger = _Logger()

    def create_publisher(self, *_args, **_kwargs):
        return self.publisher

    def get_logger(self):
        return self.logger


class _MoveIt:
    end_effector_name = 'tool0'

    def __init__(self, trajectory):
        self.trajectory = trajectory
        self.plan_calls = []
        self.plan_scalings = []
        self.executed = []
        self._states = []
        self._error = None
        self.start_execution = True
        self.max_velocity = 0.0
        self.max_acceleration = 0.0
        self.allowed_planning_time = 0.0

    def plan_async(self, **kwargs):
        self.plan_calls.append(kwargs)
        self.plan_scalings.append((self.max_velocity, self.max_acceleration))
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


class _Gripper:
    def __init__(self):
        self.cancelled = False
        self.commands = []

    def command(self, *args):
        self.commands.append(args)
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


class _RetimeClient:
    def __init__(self, response):
        self.response = response
        self.calls = 0
        self.request = None

    def wait_for_service(self, timeout_sec):
        return timeout_sec > 0.0

    def call_async(self, request):
        self.calls += 1
        self.request = request
        return _Future(self.response)


def _trajectory(times=(0, 100_000_000)):
    return SimpleNamespace(
        joint_names=list(AliciaMoveIt.JOINT_NAMES),
        points=[SimpleNamespace(
            positions=[0.0] * 6,
            time_from_start=SimpleNamespace(
                sec=t // 1_000_000_000, nanosec=t % 1_000_000_000),
        ) for t in times],
    )


def _adapter(retimed=None):
    node = _Node()
    planned = _trajectory()
    moveit = _MoveIt(planned)
    retime = _RetimeClient(SimpleNamespace(
        success=True, message='', retimed=retimed or _trajectory()))
    gripper = _Gripper()
    adapter = AliciaMoveIt(
        node, moveit=moveit, gripper=gripper, retime_client=retime,
        retime_request_factory=lambda: SimpleNamespace(
            trajectory=None, group_name='', velocity_scaling=0.0,
            acceleration_scaling=0.0), execution_timeout=0.2,
        arm_activity=_Activity(), gripper_activity=_Activity())
    return adapter, moveit, retime, gripper


def test_joint_motion_executes_planned_trajectory():
    adapter, moveit, retime, _gripper = _adapter()
    assert adapter.move_joints(AliciaMoveIt.HOME)
    assert retime.calls == 0
    assert len(moveit.executed) == 1
    assert any(
        'planned_duration=0.100s' in message and
        'velocity_scaling=0.10' in message and
        'acceleration_scaling=0.10' in message and
        'result=SUCCEEDED' in message
        for message in adapter._node.logger.infos)


def test_pose_motion_passes_observation_tolerances_to_moveit():
    adapter, moveit, _retime, _gripper = _adapter()
    assert adapter.move_pose(
        [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0],
        tolerance_position=0.02, tolerance_orientation=0.05)
    assert moveit.allowed_planning_time == pytest.approx(2.0)
    assert moveit.plan_calls[-1]['tolerance_position'] == pytest.approx(0.02)
    assert moveit.plan_calls[-1]['tolerance_orientation'] == pytest.approx(0.05)


def test_pose_motion_uses_per_call_scaling_and_restores_defaults():
    adapter, moveit, _retime, _gripper = _adapter()
    assert adapter.move_pose(
        [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0],
        max_velocity=0.4, max_acceleration=0.5)
    assert moveit.plan_scalings[-1] == pytest.approx((0.4, 0.5))
    assert moveit.max_velocity == pytest.approx(0.1)
    assert moveit.max_acceleration == pytest.approx(0.1)


def test_cartesian_pose_retimes_once_with_per_call_scaling():
    retimed = _trajectory((0, 200_000_000))
    adapter, moveit, retime, _gripper = _adapter(retimed)
    assert adapter.move_pose(
        [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0],
        cartesian=True, cartesian_max_step=0.01,
        max_velocity=0.4, max_acceleration=0.5)
    assert moveit.plan_calls[-1]['cartesian']
    assert moveit.plan_calls[-1]['max_step'] == pytest.approx(0.01)
    assert retime.calls == 1
    assert retime.request.group_name == 'arm'
    assert retime.request.velocity_scaling == pytest.approx(0.4)
    assert retime.request.acceleration_scaling == pytest.approx(0.5)
    assert moveit.executed == [retimed]


def test_cartesian_pose_refuses_failed_retime_response():
    adapter, moveit, retime, _gripper = _adapter()
    retime.response = SimpleNamespace(
        success=False, message='rejected', retimed=_trajectory())
    assert not adapter.move_pose(
        [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0], cartesian=True)
    assert retime.calls == 1
    assert moveit.executed == []


@pytest.mark.parametrize('velocity,acceleration', [
    (0.0, 0.5), (1.01, 0.5), (0.5, 0.0), (0.5, 1.01),
])
def test_invalid_motion_scaling_is_rejected(velocity, acceleration):
    node = _Node()
    with pytest.raises(ValueError):
        AliciaMoveIt(
            node, moveit=_MoveIt(_trajectory()), gripper=_Gripper(),
            velocity_scaling=velocity, acceleration_scaling=acceleration,
            arm_activity=_Activity(), gripper_activity=_Activity())


@pytest.mark.parametrize('velocity,acceleration', [
    (0.0, 0.5), (1.01, 0.5), (0.5, 0.0), (0.5, 1.01),
])
def test_invalid_per_call_scaling_is_rejected(velocity, acceleration):
    adapter, _moveit, _retime, _gripper = _adapter()
    with pytest.raises(ValueError):
        adapter.move_pose(
            [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0],
            max_velocity=velocity, max_acceleration=acceleration)


def test_gripper_open_close_and_explicit_position_are_configurable():
    adapter, _moveit, _retime, gripper = _adapter()
    assert adapter.close_gripper()
    assert adapter.open_gripper()
    assert adapter.control_gripper(position=0.02)
    assert gripper.commands == [
        (-0.05, 5.0, 0.2),
        (0.0, 5.0, 0.2),
        (0.02, 5.0, 0.2),
    ]


def test_cancel_locks_out_inflight_operation_epoch():
    adapter, moveit, _retime, _gripper = _adapter()
    adapter.state.stop()
    adapter.cancel()
    assert not adapter.move_joints(AliciaMoveIt.HOME)
    assert moveit.executed == []


def test_stale_success_is_ignored_until_new_execution_really_starts():
    adapter, moveit, _retime, _gripper = _adapter()
    moveit._error = SimpleNamespace(val=MoveItErrorCodes.SUCCESS)
    moveit.start_execution = False
    assert not adapter.move_joints(AliciaMoveIt.HOME)
    assert len(moveit.executed) == 1
    assert any(
        'planned_duration=0.100s' in message and 'result=NOT_STARTED' in message
        for message in adapter._node.logger.infos)


def test_cancel_and_wait_checks_arm_and_gripper_are_idle():
    adapter, moveit, _retime, _gripper = _adapter()
    moveit._states = [MoveIt2State.EXECUTING, MoveIt2State.IDLE]
    assert adapter.cancel_and_wait(timeout=0.2)
    assert adapter._gripper.cancelled
    assert adapter._node.publisher.messages == ['stop']
    assert adapter._arm_activity.waits == 1
    assert adapter._gripper_activity.waits == 1

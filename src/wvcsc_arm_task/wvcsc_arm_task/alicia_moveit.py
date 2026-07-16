"""Small project-owned adapter around the upstream pymoveit2 API."""

import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray
from control_msgs.action import GripperCommand
from moveit_msgs.msg import MoveItErrorCodes
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_action_status_default
from std_msgs.msg import String

from .motion_state import MotionControlState
from .trajectory_validation import valid_retimed_trajectory

try:
    from trajectory_retime_server.srv import RetimeTrajectory
except ImportError:  # Allows source-only unit tests before ROS interfaces are built.
    RetimeTrajectory = None


class _GripperClient:
    def __init__(self, node, action_name, callback_group=None):
        self._client = ActionClient(
            node, GripperCommand, action_name, callback_group=callback_group)
        self._goal_handle = None
        self._mutex = threading.Lock()
        self._active = False
        self._cancel_requested = False

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def command(self, position, max_effort, timeout):
        if not self._client.wait_for_server(timeout_sec=timeout):
            return False

        with self._mutex:
            self._active = True
            self._cancel_requested = False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        try:
            goal_future = self._client.send_goal_async(goal)
            if not self._wait_future(goal_future, timeout):
                return False
            goal_handle = goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                return False

            with self._mutex:
                self._goal_handle = goal_handle
                cancel_requested = self._cancel_requested
            if cancel_requested:
                goal_handle.cancel_goal_async()
            result_future = goal_handle.get_result_async()
            if not self._wait_future(result_future, timeout):
                self.cancel()
                return False
            result = result_future.result()
            return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        finally:
            with self._mutex:
                self._goal_handle = None
                self._active = False

    def cancel(self):
        with self._mutex:
            self._cancel_requested = True
            goal_handle = self._goal_handle
        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def wait_idle(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._mutex:
                if not self._active:
                    return True
            time.sleep(0.01)
        with self._mutex:
            return not self._active


class _ActionStatusTracker:
    """Observe whether any goal on an action server is still active."""

    ACTIVE_STATUSES = {
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    }

    def __init__(self, node, topic, callback_group=None):
        self._mutex = threading.Lock()
        self._active = False
        self._subscription = node.create_subscription(
            GoalStatusArray,
            topic,
            self._status_callback,
            qos_profile_action_status_default,
            callback_group=callback_group,
        )

    def _status_callback(self, message):
        active = any(
            status.status in self.ACTIVE_STATUSES for status in message.status_list)
        with self._mutex:
            self._active = active

    def wait_idle(self, timeout, settle_time=0.1):
        """Require an idle status continuously for a short cancellation settle time."""
        deadline = time.monotonic() + timeout
        idle_since = None
        while time.monotonic() < deadline:
            with self._mutex:
                active = self._active
            if active:
                idle_since = None
            elif idle_since is None:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since >= settle_time:
                return True
            time.sleep(0.01)
        return False


class AliciaMoveIt:
    """Synchronous task-level operations built only on public pymoveit2 APIs."""

    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def __init__(
            self, node, base_frame='alicia_base_link', group_name='arm',
            tool_link='tool0', velocity_scaling=0.1, acceleration_scaling=0.1,
            retime_timeout=5.0, execution_timeout=60.0, planning_time=2.0,
            gripper_action='/gripper_controller/gripper_cmd',
            gripper_open_position=0.0, gripper_closed_position=-0.05,
            gripper_max_effort=5.0, callback_group=None,
            state=None, moveit=None, retime_client=None, gripper=None,
            retime_request_factory=None, arm_activity=None,
            gripper_activity=None):
        if not 0.0 < float(velocity_scaling) <= 1.0:
            raise ValueError('velocity_scaling must be in (0, 1]')
        if not 0.0 < float(acceleration_scaling) <= 1.0:
            raise ValueError('acceleration_scaling must be in (0, 1]')
        if float(planning_time) <= 0.0:
            raise ValueError('planning_time must be positive')

        self._node = node
        self._group_name = group_name
        self._velocity_scaling = float(velocity_scaling)
        self._acceleration_scaling = float(acceleration_scaling)
        self._retime_timeout = float(retime_timeout)
        self._execution_timeout = float(execution_timeout)
        self._planning_time = float(planning_time)
        self._gripper_open_position = float(gripper_open_position)
        self._gripper_closed_position = float(gripper_closed_position)
        self._gripper_max_effort = float(gripper_max_effort)
        self.state = state or MotionControlState()
        self._cancel_mutex = threading.Lock()
        self._cancel_epoch = 0

        self._moveit = moveit or MoveIt2(
            node=node,
            joint_names=self.JOINT_NAMES,
            base_link_name=base_frame,
            end_effector_name=tool_link,
            group_name=group_name,
            ignore_new_calls_while_executing=True,
            callback_group=callback_group,
        )
        self._moveit.max_velocity = self._velocity_scaling
        self._moveit.max_acceleration = self._acceleration_scaling
        self._moveit.allowed_planning_time = self._planning_time
        self._trajectory_event_pub = node.create_publisher(
            String, '/trajectory_execution_event', 1)

        if retime_client is None:
            if RetimeTrajectory is None:
                raise RuntimeError('trajectory_retime_server interfaces are unavailable')
            retime_client = node.create_client(
                RetimeTrajectory, '/retime_trajectory', callback_group=callback_group)
        self._retime_client = retime_client
        self._retime_request_factory = (
            retime_request_factory or RetimeTrajectory.Request)
        self._gripper = gripper or _GripperClient(
            node, gripper_action, callback_group=callback_group)
        self._arm_activity = arm_activity or _ActionStatusTracker(
            node, '/execute_trajectory/_action/status', callback_group)
        gripper_status_topic = (
            gripper_action.rstrip('/') + '/_action/status')
        self._gripper_activity = gripper_activity or _ActionStatusTracker(
            node, gripper_status_topic, callback_group)

    def _epoch(self):
        with self._cancel_mutex:
            return self._cancel_epoch

    def _allowed(self, epoch, allow_locked):
        return (allow_locked or not self.state.locked) and epoch == self._epoch()

    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done()

    def _plan(self, timeout=None, **kwargs):
        cartesian_fraction_threshold = float(
            kwargs.pop('cartesian_fraction_threshold', 0.0))
        future = self._moveit.plan_async(**kwargs)
        timeout = self._execution_timeout if timeout is None else float(timeout)
        if future is None or not self._wait_future(future, timeout):
            return None
        return self._moveit.get_trajectory(
            future,
            cartesian=bool(kwargs.get('cartesian', False)),
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )

    def _execute(self, trajectory, epoch, allow_locked):
        if trajectory is None or not self._allowed(epoch, allow_locked):
            return False
        self._moveit.execute(trajectory)
        deadline = time.monotonic() + self._execution_timeout
        acceptance_deadline = time.monotonic() + min(5.0, self._execution_timeout)
        saw_motion = False
        while time.monotonic() < deadline:
            state = self._moveit.query_state()
            if state != MoveIt2State.IDLE:
                saw_motion = True
            elif saw_motion:
                error = self._moveit.get_last_execution_error_code()
                return error is not None and error.val == MoveItErrorCodes.SUCCESS
            if not saw_motion and time.monotonic() >= acceptance_deadline:
                return False
            if not self._allowed(epoch, allow_locked):
                return False
            time.sleep(0.01)
        self.cancel()
        return False

    def move_joints(self, positions, allow_locked=False):
        if len(positions) != len(self.JOINT_NAMES):
            raise ValueError('Alicia-M requires exactly six joint positions')
        epoch = self._epoch()
        if not self._allowed(epoch, allow_locked):
            return False
        trajectory = self._plan(
            joint_positions=[float(value) for value in positions],
            joint_names=self.JOINT_NAMES,
        )
        return self._execute(trajectory, epoch, allow_locked)

    def move_pose(
            self, position, quat_xyzw, frame_id=None, allow_locked=False,
            tolerance_position=0.001, tolerance_orientation=0.001):
        epoch = self._epoch()
        if not self._allowed(epoch, allow_locked):
            return False
        trajectory = self._plan(
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            target_link=self._moveit.end_effector_name,
            tolerance_position=float(tolerance_position),
            tolerance_orientation=float(tolerance_orientation),
        )
        return self._execute(trajectory, epoch, allow_locked)

    def move_cartesian(
            self, position, quat_xyzw, frame_id=None, max_step=0.0025,
            fraction_threshold=1.0, allow_locked=False):
        """Plan, retime exactly once, validate, then execute."""
        epoch = self._epoch()
        if not self._allowed(epoch, allow_locked):
            return False
        planned = self._plan(
            position=position,
            quat_xyzw=quat_xyzw,
            frame_id=frame_id,
            target_link=self._moveit.end_effector_name,
            cartesian=True,
            max_step=float(max_step),
            cartesian_fraction_threshold=float(fraction_threshold),
        )
        if planned is None or not self._allowed(epoch, allow_locked):
            return False
        retimed = self._retime(planned)
        if not valid_retimed_trajectory(retimed):
            self._node.get_logger().error(
                'Retiming failed or returned an invalid trajectory; execution refused')
            return False
        return self._execute(retimed, epoch, allow_locked)

    def _retime(self, trajectory):
        if not self._retime_client.wait_for_service(timeout_sec=self._retime_timeout):
            return None
        request = self._retime_request_factory()
        request.trajectory = trajectory
        request.group_name = self._group_name
        request.velocity_scaling = self._velocity_scaling
        request.acceleration_scaling = self._acceleration_scaling
        future = self._retime_client.call_async(request)
        if not self._wait_future(future, self._retime_timeout):
            return None
        response = future.result()
        if response is None or not response.success:
            return None
        return response.retimed

    def open_gripper(self, allow_locked=False):
        return self._move_gripper(self._gripper_open_position, allow_locked)

    def close_gripper(self, allow_locked=False):
        return self._move_gripper(self._gripper_closed_position, allow_locked)

    def _move_gripper(self, position, allow_locked):
        epoch = self._epoch()
        if not self._allowed(epoch, allow_locked):
            return False
        success = self._gripper.command(
            position, self._gripper_max_effort, self._execution_timeout)
        return success and self._allowed(epoch, allow_locked)

    def cancel(self):
        """Cancel arm execution globally and the gripper goal owned by this adapter."""
        with self._cancel_mutex:
            self._cancel_epoch += 1
        self._trajectory_event_pub.publish(String(data='stop'))
        self._gripper.cancel()

    def cancel_and_wait(self, timeout=None):
        """Cancel arm and gripper, then verify both public execution states are idle."""
        timeout = self._execution_timeout if timeout is None else float(timeout)
        self.cancel()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._moveit.query_state() == MoveIt2State.IDLE:
                remaining = max(0.0, deadline - time.monotonic())
                if not self._gripper.wait_idle(remaining):
                    return False
                remaining = max(0.0, deadline - time.monotonic())
                if not self._arm_activity.wait_idle(remaining):
                    return False
                remaining = max(0.0, deadline - time.monotonic())
                return self._gripper_activity.wait_idle(remaining)
            time.sleep(0.01)
        return False

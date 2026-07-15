"""MoveIt-based simulated spraying Action for Alicia-M."""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wvcsc_interfaces.action import AlignTarget, ExecuteSpray, Spray

from .motion_control import normalize_command
from .motion_state import MotionControlState
from .node_parameters import create_alicia_moveit


class SprayTask(Node):
    def __init__(self):
        super().__init__('wvcsc_spray_task')
        self._declare_parameters()
        self._home = self._joint_parameter('home_pose')
        self._observe_left = self._joint_parameter('observe_left_pose')
        self._observe_right = self._joint_parameter('observe_right_pose')
        self._min_duration = float(self.get_parameter('min_spray_duration').value)
        self._max_duration = float(self.get_parameter('max_spray_duration').value)
        self._use_vision_alignment = bool(
            self.get_parameter('use_vision_alignment').value)
        self._use_spray_action = bool(
            self.get_parameter('use_spray_action').value)
        self._vision_timeout = float(
            self.get_parameter('vision_timeout_sec').value)
        self._downstream_server_timeout = float(
            self.get_parameter('downstream_server_timeout_sec').value)
        self._downstream_margin = float(
            self.get_parameter('downstream_result_margin_sec').value)
        self.state = MotionControlState()
        self.arm, self._callback_group = create_alicia_moveit(self, self.state)
        self._vision_client = ActionClient(
            self,
            AlignTarget,
            str(self.get_parameter('vision_action_name').value),
            callback_group=self._callback_group,
        )
        self._spray_client = ActionClient(
            self,
            Spray,
            str(self.get_parameter('spray_action_name').value),
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            ExecuteSpray,
            '/arm/execute_spray',
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._legacy_service = self.create_service(
            Trigger,
            '/arm/execute_spray_legacy',
            self._execute_legacy,
            callback_group=self._callback_group,
        )
        self.command_sub = self.create_subscription(
            String,
            '/motion_control/command',
            self._on_motion_command,
            10,
            callback_group=self._callback_group,
        )
        self._abort = threading.Event()
        self._busy_mutex = threading.Lock()
        self._busy = False

    def _declare_parameters(self):
        parameters = {
            'home_pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'observe_left_pose': [
                1.886845, -1.463996, -1.033531,
                0.597978, 1.272105, -2.261712,
            ],
            'observe_right_pose': [
                -1.882066, -1.471510, -1.031065,
                -0.585215, 1.288457, -0.891742,
            ],
            'legacy_spray_side': 'left',
            'legacy_spray_duration': 2.0,
            'min_spray_duration': 0.2,
            'max_spray_duration': 10.0,
            'use_vision_alignment': False,
            'vision_action_name': '/vision/align_target',
            'vision_timeout_sec': 8.0,
            'use_spray_action': False,
            'spray_action_name': '/spray/execute',
            'downstream_server_timeout_sec': 2.0,
            'downstream_result_margin_sec': 2.0,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _joint_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{name} must contain six finite joint positions')
        return values

    def _on_motion_command(self, message):
        command = normalize_command(message.data)
        if command in ('stop', 'reset'):
            self.state.stop()
            self._abort.set()
            self.arm.cancel()
        elif command == 'resume':
            self.state.resume()
            if not self._is_busy():
                self._abort.clear()

    def _goal_callback(self, request):
        error = self._validate_goal(
            request.mission_id, request.tree_id,
            request.spray_side, request.spray_duration)
        if error:
            self.get_logger().warn(f'[ARM] rejected goal: {error}')
            return GoalResponse.REJECT
        if not self._claim():
            self.get_logger().warn('[ARM] rejected goal: busy or motion locked')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.state.stop()
        self._abort.set()
        self.arm.cancel()
        return CancelResponse.ACCEPT

    def _execute_action(self, goal_handle):
        request = goal_handle.request
        result = ExecuteSpray.Result()
        try:
            code, message = self._run_sequence(
                request.spray_side,
                float(request.spray_duration),
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback=lambda phase, progress, text: self._feedback(
                    goal_handle, phase, progress, text),
                mission_id=request.mission_id,
                tree_id=request.tree_id,
            )
            result.success = code == ExecuteSpray.Result.OK
            result.error_code = code
            result.message = message
            if result.success:
                goal_handle.succeed()
            elif code == ExecuteSpray.Result.CANCELED and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as error:
            self.get_logger().error(f'[ARM] internal error: {error}')
            result.success = False
            result.error_code = ExecuteSpray.Result.INTERNAL_ERROR
            result.message = str(error)
            goal_handle.abort()
            return result
        finally:
            self._release()

    def _run_sequence(
            self, side, duration, cancel_requested, feedback,
            mission_id='legacy', tree_id='legacy'):
        observe = self._observe_right if side == 'right' else self._observe_left
        feedback(ExecuteSpray.Feedback.MOVING_TO_OBSERVE, 0.1, 'MOVING_TO_OBSERVE')
        if not self._move(observe):
            if self._aborted(cancel_requested):
                return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
            if not self._move(self._home):
                return ExecuteSpray.Result.HOME_FAILED, 'observe and HOME motion failed'
            return ExecuteSpray.Result.OBSERVE_FAILED, 'observation motion failed'

        if self._use_vision_alignment:
            feedback(ExecuteSpray.Feedback.ALIGNING, 0.35, 'ALIGNING')
            ok, canceled, message = self._align_target(
                mission_id, tree_id, cancel_requested)
            if not ok:
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                if message.startswith('[SAFETY]'):
                    self.state.stop()
                    self._abort.set()
                    self.arm.cancel()
                    return ExecuteSpray.Result.INTERNAL_ERROR, message
                feedback(
                    ExecuteSpray.Feedback.RETURNING_HOME,
                    0.8,
                    'RETURNING_HOME',
                )
                if not self._move(self._home):
                    return ExecuteSpray.Result.HOME_FAILED, (
                        f'{message}; HOME motion failed')
                return ExecuteSpray.Result.VISION_FAILED, message

        feedback(ExecuteSpray.Feedback.SPRAYING, 0.5, 'SPRAYING')
        if self._use_spray_action:
            ok, canceled, message = self._spray_target(
                mission_id, tree_id, duration, cancel_requested)
            if not ok:
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                feedback(
                    ExecuteSpray.Feedback.RETURNING_HOME,
                    0.8,
                    'RETURNING_HOME',
                )
                if not self._move(self._home):
                    return ExecuteSpray.Result.HOME_FAILED, (
                        f'{message}; HOME motion failed')
                return ExecuteSpray.Result.SPRAY_FAILED, message
        else:
            canceled = not self._run_timer_spray(duration, cancel_requested)
            if canceled:
                return ExecuteSpray.Result.CANCELED, 'spray goal canceled'

        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        feedback(ExecuteSpray.Feedback.RETURNING_HOME, 0.8, 'RETURNING_HOME')
        if not self._move(self._home):
            if self._aborted(cancel_requested):
                return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
            return ExecuteSpray.Result.HOME_FAILED, 'HOME motion failed'
        feedback(ExecuteSpray.Feedback.COMPLETED, 1.0, 'COMPLETED')
        return ExecuteSpray.Result.OK, 'spray sequence completed at HOME'

    def _run_timer_spray(self, duration, cancel_requested):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return False
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return not self._aborted(cancel_requested)

    def _align_target(self, mission_id, tree_id, cancel_requested):
        goal = AlignTarget.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.timeout = self._vision_timeout
        wrapped, canceled, error = self._run_downstream_action(
            self._vision_client,
            goal,
            self._vision_timeout + self._downstream_margin,
            cancel_requested,
            'vision alignment',
        )
        if wrapped is None:
            return False, canceled, error
        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        return ok, False, result.message or f'vision status={wrapped.status}'

    def _spray_target(
            self, mission_id, tree_id, duration, cancel_requested):
        goal = Spray.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.duration = duration
        goal.mode = 'continuous'
        wrapped, canceled, error = self._run_downstream_action(
            self._spray_client,
            goal,
            duration + self._downstream_margin,
            cancel_requested,
            'spray actuator',
        )
        if wrapped is None:
            return False, canceled, error
        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        return ok, False, result.message or f'spray status={wrapped.status}'

    def _run_downstream_action(
            self, client, goal, result_timeout, cancel_requested, label):
        deadline = time.monotonic() + self._downstream_server_timeout
        while not client.server_is_ready():
            if self._aborted(cancel_requested):
                return None, True, f'{label} canceled'
            if time.monotonic() >= deadline:
                return None, False, f'{label} server is unavailable'
            time.sleep(0.02)

        response_future = client.send_goal_async(goal)
        response, canceled = self._wait_future(
            response_future,
            self._downstream_server_timeout,
            cancel_requested,
        )
        if response is None:
            if canceled:
                response_future.add_done_callback(self._cancel_late_goal)
            return None, canceled, f'{label} goal response timed out or canceled'
        if not response.accepted:
            return None, False, f'{label} goal was rejected'

        result_future = response.get_result_async()
        wrapped, canceled = self._wait_future(
            result_future,
            result_timeout,
            cancel_requested,
            cancel_handle=response,
        )
        if wrapped is None:
            return None, canceled, f'{label} result timed out or canceled'
        return wrapped, False, ''

    def _wait_future(
            self, future, timeout, cancel_requested, cancel_handle=None):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._aborted(cancel_requested):
                if cancel_handle is not None:
                    self._cancel_downstream_and_wait(cancel_handle, future)
                return None, True
            if time.monotonic() >= deadline:
                if cancel_handle is not None:
                    self._cancel_downstream_and_wait(cancel_handle, future)
                return None, False
            time.sleep(0.02)
        try:
            return future.result(), False
        except Exception:
            return None, False

    def _cancel_downstream_and_wait(self, goal_handle, result_future):
        """Bound cancellation so a parent result does not leave an active child."""
        deadline = time.monotonic() + self._downstream_server_timeout
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            return False
        while not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return result_future.done()

    @staticmethod
    def _cancel_late_goal(future):
        try:
            handle = future.result()
        except Exception:
            return
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    def _move(self, positions):
        return not self._abort.is_set() and self.arm.move_joints(positions)

    def _aborted(self, cancel_requested):
        return self._abort.is_set() or cancel_requested()

    @staticmethod
    def _feedback(goal_handle, phase, progress, text):
        message = ExecuteSpray.Feedback()
        message.phase = phase
        message.progress = progress
        message.phase_text = text
        goal_handle.publish_feedback(message)

    def _execute_legacy(self, _request, response):
        side = str(self.get_parameter('legacy_spray_side').value).lower()
        duration = float(self.get_parameter('legacy_spray_duration').value)
        error = self._validate_goal('legacy', 'legacy', side, duration)
        if error:
            response.message = error
            return response
        if not self._claim():
            response.message = 'spray task is busy or motion is locked'
            return response
        threading.Thread(
            target=self._run_legacy, args=(side, duration), daemon=True).start()
        response.success = True
        response.message = f'{side} legacy spray sequence accepted'
        return response

    def _run_legacy(self, side, duration):
        try:
            code, message = self._run_sequence(
                side, duration, cancel_requested=lambda: False,
                feedback=lambda *_args: None,
                mission_id='legacy', tree_id='legacy')
            log = (
                self.get_logger().info
                if code == ExecuteSpray.Result.OK
                else self.get_logger().error
            )
            log(f'[ARM] legacy result code={code}: {message}')
        finally:
            self._release()

    def _validate_goal(self, mission_id, tree_id, side, duration):
        if not str(mission_id).strip() or not str(tree_id).strip():
            return 'mission_id and tree_id are required'
        if side not in ('left', 'right'):
            return 'spray_side must be left or right'
        if not math.isfinite(duration) or not self._min_duration <= duration <= self._max_duration:
            return 'spray_duration out of range'
        return ''

    def _claim(self):
        with self._busy_mutex:
            if self._busy or self.state.locked:
                return False
            self._busy = True
            self._abort.clear()
            return True

    def _release(self):
        with self._busy_mutex:
            self._busy = False
            if not self.state.locked:
                self._abort.clear()

    def _is_busy(self):
        with self._busy_mutex:
            return self._busy


def main():
    rclpy.init()
    node = SprayTask()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

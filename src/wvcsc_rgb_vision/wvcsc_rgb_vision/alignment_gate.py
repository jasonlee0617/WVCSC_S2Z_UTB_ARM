"""Action boundary that requires fresh, centered RGB detections."""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from wvcsc_interfaces.action import AlignTarget
from wvcsc_interfaces.msg import Target2D

from .core import AlignmentTracker


class AlignmentGate(Node):
    def __init__(self, **kwargs):
        super().__init__('wvcsc_alignment_gate', **kwargs)
        parameters = {
            'target_topic': '/vision/target',
            'action_name': '/vision/align_target',
            'tolerance_u': 20.0,
            'tolerance_v': 20.0,
            'min_confidence': 0.7,
            'stable_frames': 5,
            'stale_timeout_sec': 0.5,
            'min_goal_timeout_sec': 0.5,
            'max_goal_timeout_sec': 30.0,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)
        self._min_timeout = float(
            self.get_parameter('min_goal_timeout_sec').value)
        self._max_timeout = float(
            self.get_parameter('max_goal_timeout_sec').value)
        self._tracker = AlignmentTracker(
            self.get_parameter('tolerance_u').value,
            self.get_parameter('tolerance_v').value,
            self.get_parameter('min_confidence').value,
            self.get_parameter('stable_frames').value,
            self.get_parameter('stale_timeout_sec').value,
        )
        self._busy_lock = threading.Lock()
        self._busy = False
        self.create_subscription(
            Target2D,
            str(self.get_parameter('target_topic').value),
            self._on_target,
            10,
        )
        self._server = ActionServer(
            self,
            AlignTarget,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_target(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self._tracker.update(
            stamp,
            message.mission_id,
            message.tree_id,
            message.valid,
            float(message.confidence),
            float(message.center_u),
            float(message.center_v),
            int(message.image_width),
            int(message.image_height),
        )

    def _goal(self, request):
        timeout = float(request.timeout)
        if (not request.mission_id.strip() or not request.tree_id.strip() or
                not math.isfinite(timeout) or
                not self._min_timeout <= timeout <= self._max_timeout):
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = AlignTarget.Result()
        self._tracker.reset()
        started_stamp = self._now()
        deadline = time.monotonic() + float(goal_handle.request.timeout)
        last_status = AlignmentTracker.STALE
        error_u = 0.0
        error_v = 0.0
        try:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    result.error_code = AlignTarget.Result.CANCELED
                    result.message = 'alignment canceled'
                    goal_handle.canceled()
                    return result
                last_status, error_u, error_v, stable_frames = (
                    self._tracker.status(
                        self._now(), goal_handle.request.mission_id,
                        goal_handle.request.tree_id,
                        started_stamp))
                feedback = AlignTarget.Feedback()
                feedback.phase = (
                    AlignTarget.Feedback.ALIGNED
                    if last_status == AlignmentTracker.ALIGNED
                    else AlignTarget.Feedback.ACQUIRING)
                feedback.error_u = error_u
                feedback.error_v = error_v
                feedback.stable_frames = stable_frames
                goal_handle.publish_feedback(feedback)
                if last_status == AlignmentTracker.ALIGNED:
                    result.success = True
                    result.error_code = AlignTarget.Result.OK
                    result.message = 'target alignment confirmed'
                    result.final_error_u = error_u
                    result.final_error_v = error_v
                    goal_handle.succeed()
                    return result
                time.sleep(0.02)
            result.error_code = (
                AlignTarget.Result.TARGET_STALE
                if last_status == AlignmentTracker.STALE
                else AlignTarget.Result.TIMEOUT)
            result.message = f'alignment failed: {last_status}'
            result.final_error_u = error_u
            result.final_error_v = error_v
            goal_handle.abort()
            return result
        finally:
            with self._busy_lock:
                self._busy = False


def main():
    rclpy.init()
    node = AlignmentGate()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node._server.destroy()
        node.destroy_node()
        rclpy.try_shutdown()

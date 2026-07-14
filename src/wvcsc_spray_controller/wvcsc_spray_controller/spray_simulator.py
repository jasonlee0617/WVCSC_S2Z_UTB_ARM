"""Simulation-only Spray Action server with cancel and emergency-stop interlocks."""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from wvcsc_interfaces.action import Spray

from .core import SprayInterlock


class SpraySimulator(Node):
    def __init__(self, **kwargs):
        super().__init__('wvcsc_spray_simulator', **kwargs)
        self.declare_parameter('action_name', '/spray/execute')
        self.declare_parameter('active_topic', '/spray/simulated_active')
        self.declare_parameter('emergency_stop_topic', '/safety/emergency_stop')
        self.declare_parameter('min_duration', 0.2)
        self.declare_parameter('max_duration', 10.0)
        self._interlock = SprayInterlock(
            self.get_parameter('min_duration').value,
            self.get_parameter('max_duration').value,
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._active_pub = self.create_publisher(
            Bool, str(self.get_parameter('active_topic').value), latched)
        self.create_subscription(
            Bool,
            str(self.get_parameter('emergency_stop_topic').value),
            self._on_emergency_stop,
            latched,
        )
        self._server = ActionServer(
            self,
            Spray,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self._set_active(False)

    def _goal(self, request):
        error = self._interlock.validate(
            request.mission_id,
            request.tree_id,
            float(request.duration),
            request.mode,
        )
        if error:
            self.get_logger().warn(f'[SPRAY] rejected goal: {error}')
            return GoalResponse.REJECT
        if not self._interlock.claim():
            self.get_logger().warn('[SPRAY] rejected goal: busy or emergency stopped')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = Spray.Result()
        started = time.monotonic()
        duration = float(goal_handle.request.duration)
        self._set_active(True)
        try:
            while True:
                elapsed = time.monotonic() - started
                if self._interlock.emergency_stopped:
                    result.error_code = Spray.Result.EMERGENCY_STOPPED
                    result.message = 'spray stopped by emergency interlock'
                    goal_handle.abort()
                    break
                if goal_handle.is_cancel_requested:
                    result.error_code = Spray.Result.CANCELED
                    result.message = 'spray canceled'
                    goal_handle.canceled()
                    break
                if elapsed >= duration:
                    result.success = True
                    result.error_code = Spray.Result.OK
                    result.message = 'simulated spray completed'
                    goal_handle.succeed()
                    break
                feedback = Spray.Feedback()
                feedback.phase = Spray.Feedback.ACTIVE
                feedback.elapsed = elapsed
                feedback.progress = min(1.0, elapsed / duration)
                goal_handle.publish_feedback(feedback)
                time.sleep(min(0.02, duration - elapsed))
            result.actual_duration = time.monotonic() - started
            return result
        finally:
            self._set_active(False)
            self._interlock.release()

    def _on_emergency_stop(self, message):
        self._interlock.set_emergency_stop(message.data)
        if message.data:
            self._set_active(False)

    def _set_active(self, active):
        message = Bool()
        message.data = bool(active)
        self._active_pub.publish(message)

    def force_off(self):
        self._interlock.set_emergency_stop(True)
        self._set_active(False)


def main():
    rclpy.init()
    node = SpraySimulator()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.force_off()
        executor.shutdown(timeout_sec=2.0)
        node._server.destroy()
        node.destroy_node()
        rclpy.try_shutdown()

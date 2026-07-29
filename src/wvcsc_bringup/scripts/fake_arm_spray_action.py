#!/usr/bin/env python3
# 中文说明：实车导航/继电器集成测试使用的假机械臂 Action 服务端。
# 它只验证通道 2 继电器和任务管理器数据流，不启动 MoveIt、不移动真实机械臂。
# 该脚本不能作为真实喷洒入口，也不能替代 SprayTask 的安全状态机。
"""Simulate the arm spray Action while exercising the physical relay.

This node is only for the real five-point navigation/relay integration test.
It never starts MoveIt or moves Alicia-M.  It accepts the same
``/arm/execute_spray`` contract as ``spray_task`` and drives relay channel 2
for the requested spray duration so the route manager can be tested against
the real controller and Modbus hardware.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.srv import SetRelay


class FakeArmSprayAction(Node):
    """Return successful arm goals after a real channel-2 relay pulse."""

    def __init__(self):
        super().__init__('wvcsc_fake_arm_spray_action')
        self.declare_parameter('relay_service_name', '/relay/set')
        self.declare_parameter('relay_channel', 2)
        self.declare_parameter('relay_service_timeout_sec', 2.0)
        self._relay_service_name = str(
            self.get_parameter('relay_service_name').value)
        self._relay_channel = int(self.get_parameter('relay_channel').value)
        self._relay_timeout = float(
            self.get_parameter('relay_service_timeout_sec').value)
        if self._relay_channel != 2:
            raise ValueError('fake arm validation must use relay channel 2')
        if not math.isfinite(self._relay_timeout) or self._relay_timeout <= 0.0:
            raise ValueError('relay_service_timeout_sec must be positive')

        self._callback_group = ReentrantCallbackGroup()
        self._relay_client = self.create_client(
            SetRelay, self._relay_service_name,
            callback_group=self._callback_group)
        self._goal_count = 0
        self._server = ActionServer(
            self,
            ExecuteSpray,
            '/arm/execute_spray',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f'[FAKE_ARM] ready action=/arm/execute_spray '
            f'relay={self._relay_service_name} channel={self._relay_channel}')

    def _goal_callback(self, request):
        duration = float(request.spray_duration)
        if (not str(request.mission_id).strip()
                or not math.isfinite(duration)
                or duration <= 0.0):
            self.get_logger().error('[FAKE_ARM] rejected invalid spray goal')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def _relay_request(self, enabled, duration):
        if not self._relay_client.wait_for_service(
                timeout_sec=self._relay_timeout):
            return False, 'relay service is unavailable'
        request = SetRelay.Request()
        request.channel = self._relay_channel
        request.enabled = bool(enabled)
        request.duration = float(duration)
        try:
            future = self._relay_client.call_async(request)
        except Exception as error:
            return False, f'relay request failed: {error}'

        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(self._relay_timeout):
            return False, 'relay service timed out'
        try:
            response = future.result()
        except Exception as error:
            return False, f'relay response failed: {error}'
        if response is None or not response.success:
            detail = '' if response is None else response.message
            return False, f'relay rejected request: {detail}'
        return True, str(response.message)

    def _off_best_effort(self):
        try:
            self._relay_request(False, 0.0)
        except Exception as error:
            self.get_logger().error(f'[FAKE_ARM] relay off failed: {error}')

    def _publish_feedback(self, goal_handle, phase, progress, text):
        feedback = ExecuteSpray.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        feedback.phase_text = text
        goal_handle.publish_feedback(feedback)

    def _execute_callback(self, goal_handle):
        request = goal_handle.request
        duration = float(request.spray_duration)
        self._goal_count += 1
        sequence = self._goal_count
        self.get_logger().info(
            f'[FAKE_ARM] inspect={sequence} mission={request.mission_id} '
            f'channel=2 duration={duration:.1f}s')
        result = ExecuteSpray.Result()

        ok, message = self._relay_request(True, duration)
        if not ok:
            result.success = False
            result.error_code = ExecuteSpray.Result.SPRAY_FAILED
            result.message = message
            goal_handle.abort()
            return result

        self._publish_feedback(
            goal_handle, ExecuteSpray.Feedback.SPRAYING, 0.60,
            'FAKE_ARM_RELAY_2_ON')
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self._off_best_effort()
                result.success = False
                result.error_code = ExecuteSpray.Result.CANCELED
                result.message = 'fake arm spray canceled'
                goal_handle.canceled()
                return result
            time.sleep(0.05)

        off_ok, off_message = self._relay_request(False, 0.0)
        if not off_ok:
            result.success = False
            result.error_code = ExecuteSpray.Result.SPRAY_FAILED
            result.message = f'relay channel 2 off failed: {off_message}'
            goal_handle.abort()
            return result

        result.success = True
        result.error_code = ExecuteSpray.Result.OK
        result.message = (
            f'fake arm completed; relay channel 2 pulsed for {duration:.1f}s')
        self._publish_feedback(
            goal_handle, ExecuteSpray.Feedback.COMPLETED, 1.0,
            'FAKE_ARM_COMPLETED')
        goal_handle.succeed()
        self.get_logger().info(
            f'[FAKE_ARM] inspect={sequence} completed channel=2 OFF')
        return result


def main():
    rclpy.init()
    node = FakeArmSprayAction()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warning('[FAKE_ARM] interrupted; turning channel 2 off')
        node._off_best_effort()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

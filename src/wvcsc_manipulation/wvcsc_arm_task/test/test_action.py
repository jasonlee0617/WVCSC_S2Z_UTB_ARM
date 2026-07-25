import os
import time

os.environ['ROS_DOMAIN_ID'] = '84'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/wvcsc_spray_test_logs')

import pytest
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from wvcsc_interfaces.action import Spray
from wvcsc_interfaces.srv import SetRelay

from wvcsc_arm_task.spray_actuator import SprayActuator


def _spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def _goal(duration):
    goal = Spray.Goal()
    goal.mission_id = 'test_mission'
    goal.tree_id = 'tree_01'
    goal.duration = duration
    goal.mode = 'continuous'
    return goal


def test_success_and_cancel_always_close_simulated_actuator():
    context = Context()
    rclpy.init(context=context)
    server = SprayActuator(context=context)
    client_node = Node('spray_action_test_client', context=context)
    client = ActionClient(client_node, Spray, '/spray/execute')
    active_samples = []
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    client_node.create_subscription(
        Bool, '/spray/simulated_active',
        lambda message: active_samples.append(message.data), qos)
    motion_locked = client_node.create_publisher(
        Bool, '/motion_control/locked', qos)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    try:
        assert _spin_until(executor, client.server_is_ready)
        send = client.send_goal_async(_goal(0.2))
        assert _spin_until(executor, send.done)
        result_future = send.result().get_result_async()
        assert _spin_until(executor, result_future.done)
        assert result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        assert result_future.result().result.success
        assert True in active_samples and active_samples[-1] is False

        send = client.send_goal_async(_goal(1.0))
        assert _spin_until(executor, send.done)
        handle = send.result()
        assert handle.accepted
        assert _spin_until(
            executor, lambda: bool(active_samples) and active_samples[-1] is True)
        cancel = handle.cancel_goal_async()
        assert _spin_until(executor, cancel.done)
        result_future = handle.get_result_async()
        assert _spin_until(executor, result_future.done)
        assert result_future.result().status == GoalStatus.STATUS_CANCELED
        assert active_samples[-1] is False

        send = client.send_goal_async(_goal(1.0))
        assert _spin_until(executor, send.done)
        handle = send.result()
        assert handle.accepted
        assert _spin_until(
            executor, lambda: bool(active_samples) and active_samples[-1] is True)
        motion_locked.publish(Bool(data=True))
        result_future = handle.get_result_async()
        assert _spin_until(executor, result_future.done)
        wrapped = result_future.result()
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.error_code == Spray.Result.EMERGENCY_STOPPED
        assert active_samples[-1] is False
    finally:
        server.force_off()
        client.destroy()
        server._server.destroy()
        for node in (client_node, server):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()


def test_service_mode_confirms_channel_two_and_relay_duration():
    context = Context()
    rclpy.init(context=context)
    relay_node = Node('relay_service_test', context=context)
    requests = []

    def set_relay(request, response):
        requests.append((request.channel, request.enabled, request.duration))
        response.success = True
        response.message = 'ok'
        return response

    relay_node.create_service(SetRelay, '/relay/set', set_relay)
    server = SprayActuator(
        context=context,
        parameter_overrides=[
            Parameter('spray_mode', value='service'),
            Parameter('relay_channel', value=2),
            Parameter('relay_service_timeout_sec', value=1.0),
        ])
    client_node = Node('spray_service_test_client', context=context)
    client = ActionClient(client_node, Spray, '/spray/execute')
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    for node in (relay_node, server, client_node):
        executor.add_node(node)
    try:
        assert _spin_until(executor, client.server_is_ready)
        send = client.send_goal_async(_goal(0.2))
        assert _spin_until(executor, send.done)
        result_future = send.result().get_result_async()
        assert _spin_until(executor, result_future.done)
        wrapped = result_future.result()
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.success
        assert requests[0][:2] == (2, True)
        assert requests[0][2] == pytest.approx(0.2)
        assert requests[-1] == (2, False, 0.0)
    finally:
        server.force_off()
        client.destroy()
        server._server.destroy()
        for node in (client_node, server, relay_node):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()


def test_service_mode_rejects_a_failed_relay_request():
    context = Context()
    rclpy.init(context=context)
    relay_node = Node('relay_failure_test', context=context)

    def reject_relay(_request, response):
        response.success = False
        response.message = 'modbus write failed'
        return response

    relay_node.create_service(SetRelay, '/relay/set', reject_relay)
    server = SprayActuator(
        context=context,
        parameter_overrides=[
            Parameter('spray_mode', value='service'),
            Parameter('relay_channel', value=2),
            Parameter('relay_service_timeout_sec', value=1.0),
        ])
    client_node = Node('spray_failure_test_client', context=context)
    client = ActionClient(client_node, Spray, '/spray/execute')
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    for node in (relay_node, server, client_node):
        executor.add_node(node)
    try:
        assert _spin_until(executor, client.server_is_ready)
        send = client.send_goal_async(_goal(0.2))
        assert _spin_until(executor, send.done)
        result_future = send.result().get_result_async()
        assert _spin_until(executor, result_future.done)
        wrapped = result_future.result()
        assert wrapped.status == GoalStatus.STATUS_ABORTED
        assert wrapped.result.error_code == Spray.Result.RELAY_FAILED
        assert not wrapped.result.success
    finally:
        server.force_off()
        client.destroy()
        server._server.destroy()
        for node in (client_node, server, relay_node):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()

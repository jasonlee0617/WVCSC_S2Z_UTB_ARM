# flake8: noqa
import os
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '86'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/wvcsc_arm_chain_test_logs')

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from wvcsc_interfaces.action import AlignTarget, ExecuteSpray, Spray

import wvcsc_arm_task.spray_task as spray_task_module


class _Arm:
    def __init__(self):
        self.moves = []
        self.canceled = False

    def move_joints(self, positions):
        self.moves.append(list(positions))
        return True

    def cancel(self):
        self.canceled = True


class _DownstreamServers(Node):
    def __init__(self, hold_spray=False):
        super().__init__('optional_action_chain_servers')
        self.requests = []
        self.hold_spray = hold_spray
        self.spray_finished = threading.Event()
        group = ReentrantCallbackGroup()
        self.vision_server = ActionServer(
            self, AlignTarget, '/vision/align_target',
            execute_callback=self._align, callback_group=group)
        self.spray_server = ActionServer(
            self, Spray, '/spray/execute',
            execute_callback=self._spray,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=group)

    def _align(self, goal_handle):
        request = goal_handle.request
        self.requests.append(('align', request.mission_id, request.tree_id))
        result = AlignTarget.Result()
        result.success = True
        result.error_code = AlignTarget.Result.OK
        result.message = 'aligned'
        goal_handle.succeed()
        return result

    def _spray(self, goal_handle):
        request = goal_handle.request
        self.requests.append((
            'spray', request.mission_id, request.tree_id,
            float(request.duration), request.mode))
        result = Spray.Result()
        if self.hold_spray:
            deadline = time.monotonic() + 2.0
            while (not goal_handle.is_cancel_requested and
                   time.monotonic() < deadline):
                time.sleep(0.01)
        if goal_handle.is_cancel_requested:
            result.error_code = Spray.Result.CANCELED
            result.message = 'spray canceled'
            goal_handle.canceled()
        else:
            result.success = True
            result.error_code = Spray.Result.OK
            result.message = 'sprayed'
            goal_handle.succeed()
        result.actual_duration = request.duration
        self.spray_finished.set()
        return result

    def destroy_servers(self):
        self.vision_server.destroy()
        self.spray_server.destroy()


def _spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_execute_spray_chains_vision_and_spray_actions(monkeypatch):
    arm = _Arm()
    monkeypatch.setattr(
        spray_task_module,
        'create_alicia_moveit',
        lambda _node, _state: (arm, ReentrantCallbackGroup()),
    )
    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_vision_alignment:=true',
        '-p', 'use_spray_action:=true',
        '-p', 'vision_timeout_sec:=1.0',
        '-p', 'downstream_server_timeout_sec:=1.0',
        '-p', 'downstream_result_margin_sec:=1.0',
    ])
    task = spray_task_module.SprayTask()
    servers = _DownstreamServers()
    client_node = Node('optional_action_chain_client')
    client = ActionClient(client_node, ExecuteSpray, '/arm/execute_spray')
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (task, servers, client_node):
        executor.add_node(node)
    try:
        assert _spin_until(executor, client.server_is_ready)
        goal = ExecuteSpray.Goal()
        goal.mission_id = 'mission_01'
        goal.tree_id = 'tree_01'
        goal.spray_side = 'left'
        goal.spray_duration = 0.2
        send = client.send_goal_async(goal)
        assert _spin_until(executor, send.done)
        handle = send.result()
        assert handle.accepted
        result_future = handle.get_result_async()
        assert _spin_until(executor, result_future.done, timeout=5.0)
        wrapped = result_future.result()
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.success
        assert wrapped.result.error_code == ExecuteSpray.Result.OK
        assert [request[0] for request in servers.requests] == ['align', 'spray']
        assert all(request[1:3] == ('mission_01', 'tree_01')
                   for request in servers.requests)
        assert arm.moves == [task._observe_left, task._home]
    finally:
        client.destroy()
        task._action_server.destroy()
        servers.destroy_servers()
        for node in (client_node, servers, task):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()


def test_parent_cancel_propagates_to_active_spray_action(monkeypatch):
    arm = _Arm()
    monkeypatch.setattr(
        spray_task_module,
        'create_alicia_moveit',
        lambda _node, _state: (arm, ReentrantCallbackGroup()),
    )
    rclpy.init(args=[
        '--ros-args',
        '-p', 'use_vision_alignment:=true',
        '-p', 'use_spray_action:=true',
        '-p', 'vision_timeout_sec:=1.0',
        '-p', 'downstream_server_timeout_sec:=1.0',
        '-p', 'downstream_result_margin_sec:=1.0',
    ])
    task = spray_task_module.SprayTask()
    servers = _DownstreamServers(hold_spray=True)
    client_node = Node('optional_action_cancel_client')
    client = ActionClient(client_node, ExecuteSpray, '/arm/execute_spray')
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (task, servers, client_node):
        executor.add_node(node)
    try:
        assert _spin_until(executor, client.server_is_ready)
        goal = ExecuteSpray.Goal()
        goal.mission_id = 'mission_02'
        goal.tree_id = 'tree_02'
        goal.spray_side = 'right'
        goal.spray_duration = 1.0
        send = client.send_goal_async(goal)
        assert _spin_until(executor, send.done)
        handle = send.result()
        assert handle.accepted
        assert _spin_until(
            executor,
            lambda: any(item[0] == 'spray' for item in servers.requests),
        )
        cancel = handle.cancel_goal_async()
        assert _spin_until(executor, cancel.done)
        result_future = handle.get_result_async()
        assert _spin_until(executor, result_future.done, timeout=5.0)
        assert result_future.result().status == GoalStatus.STATUS_CANCELED
        assert _spin_until(executor, servers.spray_finished.is_set)
        assert arm.canceled
        assert arm.moves == [task._observe_right]
    finally:
        client.destroy()
        task._action_server.destroy()
        servers.destroy_servers()
        for node in (client_node, servers, task):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()

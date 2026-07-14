import os
import time

os.environ['ROS_DOMAIN_ID'] = '85'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/wvcsc_vision_test_logs')

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from wvcsc_interfaces.action import AlignTarget
from wvcsc_interfaces.msg import Target2D

from wvcsc_rgb_vision.alignment_gate import AlignmentGate


def _spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_centered_frames_complete_alignment_action():
    context = Context()
    rclpy.init(context=context)
    gate = AlignmentGate(
        context=context,
        parameter_overrides=[Parameter('stable_frames', value=3)])
    client_node = Node('alignment_test_client', context=context)
    client = ActionClient(client_node, AlignTarget, '/vision/align_target')
    publisher = client_node.create_publisher(Target2D, '/vision/target', 10)
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(gate)
    executor.add_node(client_node)
    try:
        assert _spin_until(executor, client.server_is_ready)
        goal = AlignTarget.Goal()
        goal.mission_id = 'mission'
        goal.tree_id = 'tree_01'
        goal.timeout = 1.0
        send = client.send_goal_async(goal)
        assert _spin_until(executor, send.done)
        handle = send.result()
        assert handle.accepted
        for _ in range(3):
            message = Target2D()
            message.header.stamp = client_node.get_clock().now().to_msg()
            message.mission_id = 'mission'
            message.tree_id = 'tree_01'
            message.valid = True
            message.confidence = 0.95
            message.center_u = 640.0
            message.center_v = 360.0
            message.image_width = 1280
            message.image_height = 720
            publisher.publish(message)
            executor.spin_once(timeout_sec=0.03)
        result_future = handle.get_result_async()
        assert _spin_until(executor, result_future.done)
        assert result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        assert result_future.result().result.success

        stale_goal = AlignTarget.Goal()
        stale_goal.mission_id = 'mission'
        stale_goal.tree_id = 'tree_02'
        stale_goal.timeout = 0.5
        send = client.send_goal_async(stale_goal)
        assert _spin_until(executor, send.done)
        stale_result = send.result().get_result_async()
        assert _spin_until(executor, stale_result.done)
        assert stale_result.result().status == GoalStatus.STATUS_ABORTED
        assert (stale_result.result().result.error_code ==
                AlignTarget.Result.TARGET_STALE)

        cancel_goal = AlignTarget.Goal()
        cancel_goal.mission_id = 'mission'
        cancel_goal.tree_id = 'tree_03'
        cancel_goal.timeout = 1.0
        send = client.send_goal_async(cancel_goal)
        assert _spin_until(executor, send.done)
        cancel_handle = send.result()
        cancel = cancel_handle.cancel_goal_async()
        assert _spin_until(executor, cancel.done)
        cancel_result = cancel_handle.get_result_async()
        assert _spin_until(executor, cancel_result.done)
        assert cancel_result.result().status == GoalStatus.STATUS_CANCELED
        assert (cancel_result.result().result.error_code ==
                AlignTarget.Result.CANCELED)
    finally:
        client.destroy()
        gate._server.destroy()
        for node in (client_node, gate):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()

#!/usr/bin/env python3
"""Send one Alicia-M spray goal without Nav2 or MissionManager."""

import argparse
import sys

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import MissionStatus


class ArmSprayOnce(Node):
    def __init__(self, args):
        super().__init__('wvcsc_arm_spray_once')
        self.args = args
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            MissionStatus, '/mission/status', latched)
        self.client = ActionClient(self, ExecuteSpray, '/arm/execute_spray')
        self.done = False
        self.exit_code = 1
        self.create_timer(0.5, self._publish_active_status)

    def start(self):
        if not self.client.wait_for_server(timeout_sec=self.args.timeout_sec):
            self.get_logger().error('/arm/execute_spray is unavailable')
            self.done = True
            return
        self._publish_active_status()
        goal = ExecuteSpray.Goal()
        goal.mission_id = self.args.mission_id
        goal.tree_id = self.args.target_id
        goal.spray_duration = self.args.spray_duration
        goal.tree_hint = PointStamped()
        goal.tree_hint.header.stamp = self.get_clock().now().to_msg()
        goal.tree_hint.header.frame_id = self.args.frame_id
        goal.tree_hint.point.x = self.args.tree_x_m
        goal.tree_hint.point.y = self.args.tree_y_m
        goal.tree_hint.point.z = self.args.tree_z_m
        future = self.client.send_goal_async(
            goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)
        self.get_logger().info(
            f'[ARM_TEST] sent target={goal.tree_id} '
            f'tree_hint={goal.tree_hint.header.frame_id}:'
            f'({goal.tree_hint.point.x:.2f},{goal.tree_hint.point.y:.2f},'
            f'{goal.tree_hint.point.z:.2f})')

    def _publish_active_status(self):
        if self.done:
            return
        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.frame_id
        message.mission_id = self.args.mission_id
        message.state = MissionStatus.ARM_SPRAYING
        message.state_text = 'ARM_SPRAYING'
        message.current_tree_id = self.args.target_id
        message.current_index = 0
        message.total_targets = 1
        message.arm_goal_active = True
        self.status_pub.publish(message)

    def _goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('[ARM_TEST] spray Action rejected the goal')
            self.done = True
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(
            f'[ARM_TEST] {feedback.phase_text} progress={feedback.progress:.2f}')

    def _result(self, future):
        result = future.result().result
        if result.success:
            self.exit_code = 0
            self.get_logger().info(
                f'[ARM_TEST] completed code={result.error_code}: {result.message}')
        else:
            self.get_logger().error(
                f'[ARM_TEST] failed code={result.error_code}: {result.message}')
        self.done = True
        final_status = MissionStatus()
        final_status.header.stamp = self.get_clock().now().to_msg()
        final_status.header.frame_id = self.args.frame_id
        final_status.mission_id = self.args.mission_id
        final_status.state = (
            MissionStatus.MISSION_COMPLETED if result.success else MissionStatus.FAILED)
        final_status.state_text = (
            'MISSION_COMPLETED' if result.success else 'FAILED')
        final_status.current_tree_id = ''
        final_status.total_targets = 1
        final_status.completed_targets = 1 if result.success else 0
        final_status.last_error = '' if result.success else result.message
        self.status_pub.publish(final_status)


def _args():
    argv = remove_ros_args(args=sys.argv)[1:]
    if argv and argv[0] == '--':
        argv = argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission-id', default='arm_only_spray_001')
    parser.add_argument('--target-id', default='corn_01')
    parser.add_argument('--frame-id', default='alicia_base_link')
    parser.add_argument(
        '--tree-x-m', type=float, default=0.0,
        help='tree X in --frame-id (+X is forward for alicia_base_link)')
    parser.add_argument(
        '--tree-y-m', type=float, default=1.50,
        help='tree Y in --frame-id (+Y is left; -Y is right for alicia_base_link)')
    parser.add_argument('--tree-z-m', type=float, default=0.0)
    parser.add_argument('--spray-duration', type=float, default=5.0)
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    return parser.parse_args(argv)


def main():
    args = _args()
    rclpy.init(args=sys.argv)
    node = ArmSprayOnce(args)
    try:
        node.start()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

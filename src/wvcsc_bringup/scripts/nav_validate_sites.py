#!/usr/bin/env python3
"""Validate measured docking poses by driving Nav2 to each site in sequence."""

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from wvcsc_bringup.site_mission import load_site_document


def _yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class NavValidateSites(Node):
    def __init__(self):
        super().__init__('wvcsc_nav_validate_sites')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _wait_server(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while not self._client.server_is_ready():
            if time.monotonic() >= deadline:
                raise RuntimeError('/navigate_to_pose server is not available')
            rclpy.spin_once(self, timeout_sec=0.05)

    def _navigate(self, target_label, x, y, yaw, timeout=120.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = (
            _yaw_to_quaternion(yaw))

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'{target_label}: Nav2 rejected goal')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status_text = (
                'timeout' if result is None
                else f'status={result.status}')
            raise RuntimeError(f'{target_label}: Nav2 failed ({status_text})')
        return True

    def validate(self, targets, pause_sec):
        count = len(targets)
        self.get_logger().info(f'[VALIDATE] starting {count} targets, pause={pause_sec}s')
        for index, target in enumerate(targets):
            label = target['target_id']
            pose = target['docking_pose']
            x = float(pose['x']); y = float(pose['y']); yaw = float(pose['yaw'])
            self.get_logger().info(
                f'[{index+1}/{count}] {label}: navigating to ({x:.3f},{y:.3f},{yaw:.3f})')
            self._navigate(label, x, y, yaw)
            self.get_logger().info(f'[{index+1}/{count}] {label}: arrived')
            if index < count - 1:
                self.get_logger().info(f'[{index+1}/{count}] {label}: pausing {pause_sec}s...')
                deadline = time.monotonic() + pause_sec
                while time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
            else:
                self.get_logger().info(f'[{index+1}/{count}] {label}: last target — done')
        self.get_logger().info(f'[VALIDATE] all {count} targets completed successfully')


def main():
    argv = remove_ros_args(args=sys.argv)[1:]
    if argv and argv[0] == '--':
        argv = argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', required=True)
    parser.add_argument('--pause-sec', type=float, default=2.0)
    parser.add_argument('--timeout-sec', type=float, default=120.0)
    args = parser.parse_args(argv)

    rclpy.init(args=sys.argv)
    node = NavValidateSites()
    try:
        document = load_site_document(args.file)
        node._wait_server()
        node.validate(document['mission']['targets'], args.pause_sec)
        return 0
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(f'[VALIDATE] failed: {error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

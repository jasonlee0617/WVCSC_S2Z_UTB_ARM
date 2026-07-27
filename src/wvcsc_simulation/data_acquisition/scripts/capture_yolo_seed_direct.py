#!/usr/bin/env python3
"""Capture six raw C10 frames after direct seed-specific docking poses."""

import argparse
import sys
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import MissionStatus
from wvcsc_simulation.data_acquisition.yolo_seed_dataset import write_fruit_seg_sample


TARGETS = (
    ('tree_01', 3.0, 2.0, 0.5, 0.4, 1.5),
    ('tree_02', 7.0, 2.0, 0.5, 0.4, 1.5),
    ('tree_03', 11.0, 2.0, 0.5, 0.4, 1.5),
    ('tree_04', 1.0, -2.0, -0.5, 0.4, -1.5),
    ('tree_05', 5.0, -2.0, -0.5, 0.4, -1.5),
    ('tree_06', 9.0, -2.0, -0.5, 0.4, -1.5),
)


class DirectCapture(Node):
    def __init__(self, args):
        super().__init__('wvcsc_direct_c10_capture')
        self.args = args
        self.bridge = CvBridge()
        self.pending = False
        self.captured = False
        self.set_future = None
        self.goal_future = None
        self.state = 'wait_services'
        self.index = 0
        self.ready_at = 0.0
        self.started_at = time.monotonic()
        self.error = ''
        self.done = False
        self.x = self.y = 0.0
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.mission_status = self.create_publisher(
            MissionStatus, '/mission/status', QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.set_state = self.create_client(SetEntityState, '/set_entity_state')
        self.spray = ActionClient(self, ExecuteSpray, '/arm/execute_spray')
        self.create_subscription(
            Image, '/camera/color/image_raw', self._image,
            qos_profile_sensor_data)
        self.create_timer(0.05, self._step)

    @property
    def target(self):
        return TARGETS[self.index]

    def _publish_pose(self):
        now = self.get_clock().now().to_msg()
        message = Odometry()
        message.header.stamp = now
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.w = 1.0
        self.odom.publish(message)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = message.pose.pose.orientation
        self.tf.sendTransform(transform)

    def _publish_mission_status(self):
        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mission_id = f'c10_capture_{self.args.orchard_seed}'
        message.state = MissionStatus.ARM_SPRAYING
        message.state_text = 'direct C10 capture'
        message.current_tree_id = self.target[0]
        message.current_index = self.index
        message.total_targets = len(TARGETS)
        message.arm_goal_active = self.state in {
            'wait_goal', 'action', 'settle', 'wait_pose'}
        self.mission_status.publish(message)

    def _image(self, message):
        if not self.pending or self.captured:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            tree_id, tree_x, tree_y, _dock_y, base_x, base_y = self.target
            metadata = {
                'tree_id': tree_id,
                'tree_offset_arm_base_m': {'x_m': base_x, 'y_m': base_y},
                'orchard_seed': self.args.orchard_seed,
                'diseased_fruit_ratio': self.args.diseased_fruit_ratio,
                'camera_pose': {'frame_id': 'map'},
                'tree_hint': [tree_x, tree_y, 0.0],
                'ros_timestamp': {
                    'sec': message.header.stamp.sec,
                    'nanosec': message.header.stamp.nanosec,
                },
            }
            record = write_fruit_seg_sample(
                self.args.fruit_output, image,
                f'seed_{self.args.orchard_seed}_{tree_id}',
                self.args.split, metadata)
            self.captured = True
            self.pending = False
            self.get_logger().info(f'captured {record["image"]}')
        except Exception as error:
            self.error = f'capture failed: {error}'
            self.done = True

    def _feedback(self, feedback_message):
        if feedback_message.feedback.phase == ExecuteSpray.Feedback.DETECTING_TARGETS:
            self.pending = True

    def _set_robot_pose(self):
        _tree_id, tree_x, _tree_y, dock_y, _base_x, _base_y = self.target
        self.x, self.y = tree_x, dock_y
        state = EntityState()
        state.name = 'wvcsc_utb_alicia'
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.orientation.w = 1.0
        state.reference_frame = 'world'
        self.set_future = self.set_state.call_async(
            SetEntityState.Request(state=state))
        self.state = 'wait_pose'
        self.started_at = time.monotonic()

    def _send_goal(self):
        tree_id, tree_x, tree_y, _dock_y, _base_x, _base_y = self.target
        goal = ExecuteSpray.Goal()
        goal.mission_id = f'c10_capture_{self.args.orchard_seed}'
        goal.tree_id = tree_id
        goal.spray_duration = 0.2
        goal.tree_hint = PointStamped()
        goal.tree_hint.header.frame_id = 'map'
        goal.tree_hint.point.x = tree_x
        goal.tree_hint.point.y = tree_y
        goal.working_range_m = 0.0
        self.goal_future = self.spray.send_goal_async(
            goal, feedback_callback=self._feedback)
        self.goal_future.add_done_callback(self._goal_response)
        self.state = 'wait_goal'

    def _goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.error = f'action goal failed: {error}'
            self.done = True
            return
        if not handle or not handle.accepted:
            self.error = 'spray Action rejected goal'
            self.done = True
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._action_result)
        self.state = 'action'

    def _action_result(self, future):
        try:
            result = future.result().result
        except Exception as error:
            self.error = f'action result failed: {error}'
            self.done = True
            return
        if not result.success or not self.captured:
            self.error = (
                f'{self.target[0]} action={result.message}, '
                f'captured={self.captured}')
            self.done = True
            return
        self.index += 1
        if self.index == len(TARGETS):
            self.done = True
            return
        self.pending = False
        self.captured = False
        self.state = 'set_pose'
        self.started_at = time.monotonic()

    def _step(self):
        self._publish_pose()
        if self.done:
            return
        self._publish_mission_status()
        if time.monotonic() - self.started_at > 120.0:
            self.error = f'timeout at {self.target[0]}'
            self.done = True
            return
        if self.state == 'wait_services':
            if self.set_state.service_is_ready() and self.spray.server_is_ready():
                self.state = 'set_pose'
                self.started_at = time.monotonic()
        elif self.state == 'set_pose':
            self._set_robot_pose()
        elif self.state == 'wait_pose' and self.set_future.done():
            try:
                response = self.set_future.result()
            except Exception as error:
                self.error = f'set pose failed: {error}'
                self.done = True
                return
            if not response.success:
                self.error = f'set pose rejected: {response.status_message}'
                self.done = True
                return
            self.ready_at = time.monotonic() + 1.0
            self.state = 'settle'
        elif self.state == 'settle' and time.monotonic() >= self.ready_at:
            self._send_goal()


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fruit-output', required=True)
    parser.add_argument('--orchard-seed', type=int, required=True)
    parser.add_argument('--split', choices=('train', 'val'), required=True)
    parser.add_argument('--diseased-fruit-ratio', type=float, default=0.50)
    return parser.parse_args()


def main():
    args = _arguments()
    rclpy.init()
    node = DirectCapture(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        error = node.error
        node.destroy_node()
        rclpy.try_shutdown()
    if error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Capture one C10 frame after each tree goal reaches its observation pose."""

import argparse
import threading
import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.action._execute_spray import ExecuteSpray_FeedbackMessage
from wvcsc_interfaces.msg import MissionPlan, MissionStatus
from wvcsc_simulation.data_acquisition.yolo_seed_dataset import (
    write_fruit_seg_sample,
    write_unlabeled_sample,
)


class SeedCapture(Node):
    def __init__(self, args):
        super().__init__('wvcsc_yolo_seed_capture')
        self.args = args
        self.bridge = CvBridge()
        self.current_tree = ''
        self.tree_offsets = {}
        self.pending = ''
        self.captured = set()
        self.failure = ''
        self.done = threading.Event()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Image, args.image_topic, self._image, qos_profile_sensor_data)
        self.create_subscription(
            MissionStatus, '/mission/status', self._status, 10)
        self.create_subscription(
            MissionPlan, '/mission/plan', self._plan,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            ExecuteSpray_FeedbackMessage,
            '/arm/execute_spray/_action/feedback', self._feedback, 10)

    def _status(self, message):
        self.current_tree = message.current_tree_id
        if message.state == MissionStatus.FAILED:
            self.failure = message.last_error or message.state_text
            self.done.set()

    def _plan(self, message):
        self.tree_offsets.update({
            target.target_id: {
                'x_m': target.tree_x_m,
                'y_m': target.tree_y_m,
            }
            for target in message.targets})

    def _feedback(self, message):
        if (message.feedback.phase == ExecuteSpray.Feedback.SCANNING_TREE and
                self.current_tree and self.current_tree not in self.captured):
            self.pending = self.current_tree

    def _camera_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_color_optical_frame', rclpy.time.Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'frame_id': 'map',
            'position': [translation.x, translation.y, translation.z],
            'quaternion_xyzw': [rotation.x, rotation.y, rotation.z, rotation.w],
        }

    def _image(self, message):
        tree_id = self.pending
        if not tree_id or tree_id in self.captured:
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        stamp = message.header.stamp
        metadata = {
            'tree_id': tree_id,
            'tree_offset_arm_base_m': self.tree_offsets.get(tree_id),
            'orchard_seed': self.args.orchard_seed,
            'diseased_fruit_ratio': self.args.diseased_fruit_ratio,
            'observation_distance': self.args.observation_distance,
            'camera_pose': self._camera_pose(),
            'ros_timestamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
        }
        try:
            sample_name = f'seed_{self.args.orchard_seed}_{tree_id}'
            if self.args.split == 'unlabeled':
                record = write_unlabeled_sample(
                    self.args.output, image, sample_name, metadata)
            else:
                record = write_fruit_seg_sample(
                    self.args.output, image, sample_name, self.args.split,
                    metadata)
        except (OSError, ValueError) as error:
            self.get_logger().error(f'rejected {tree_id} frame: {error}')
            return
        self.captured.add(tree_id)
        self.pending = ''
        status = 'unlabeled' if self.args.split == 'unlabeled' else 'pending annotation'
        self.get_logger().info(f'captured {record["image"]}: {status}')
        if len(self.captured) >= self.args.expected:
            self.done.set()


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--orchard-seed', type=int, required=True)
    parser.add_argument('--expected', type=int, required=True)
    parser.add_argument(
        '--split', choices=('unlabeled', 'train', 'val'), default='unlabeled')
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--diseased-fruit-ratio', type=float, default=0.20)
    parser.add_argument('--observation-distance', type=float, default=1.40)
    parser.add_argument(
        '--image-topic', default='/camera/color/image_raw')
    return parser.parse_args()


def main():
    args = _arguments()
    rclpy.init()
    node = SeedCapture(args)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.done.is_set() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.failure:
            raise RuntimeError(f'mission failed: {node.failure}')
        if len(node.captured) < args.expected:
            raise TimeoutError(
                f'captured {len(node.captured)}/{args.expected} samples')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Gazebo RGB lesion detector used until YOLO-Seg weights are available."""

from dataclasses import dataclass

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from wvcsc_interfaces.msg import MissionStatus, Target2D

from .visualization import draw_detection


@dataclass(frozen=True)
class Candidate:
    x: int
    y: int
    width: int
    height: int
    area: int
    target_u: float
    target_v: float
    confidence: float


def safest_mask_point(mask):
    """Return the pixel with maximum clearance from a segmentation boundary."""
    distance = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    _minimum, _maximum, _min_point, max_point = cv2.minMaxLoc(distance)
    return float(max_point[0]), float(max_point[1])


def select_best(candidates):
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item.confidence, item.area))


class ColorSegmentation(Node):
    def __init__(self):
        super().__init__('wvcsc_color_segmentation')
        defaults = {
            'image_topic': '/camera/camera/color/image_rect_raw',
            'detections_topic': '/vision/pest_detections',
            'target_topic': '/vision/target',
            'debug_image_topic': '/vision/debug_image',
            'publish_debug_image': True,
            'min_area_px': 400,
            'nominal_area_px': 2500,
            'red_hue_low_1': 14,
            'red_hue_high_1': 25,
            'red_hue_low_2': 14,
            'red_hue_high_2': 25,
            'saturation_min': 120,
            'value_min': 35,
            'morphology_kernel': 5,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._bridge = CvBridge()
        self._mission_id = ''
        self._tree_id = ''
        self._reported_tree_id = ''
        self._last_mission_state = None
        self._image_frames = 0
        self._status_group = MutuallyExclusiveCallbackGroup()
        self._image_group = MutuallyExclusiveCallbackGroup()
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_subscription = self.create_subscription(
            MissionStatus, '/mission/status', self._on_status, latched,
            callback_group=self._status_group)
        self._image_subscription = self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data,
            callback_group=self._image_group)
        self._detections = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('detections_topic').value), 10)
        self._target = self.create_publisher(
            Target2D, str(self.get_parameter('target_topic').value), 10)
        self._debug = self.create_publisher(
            Image, str(self.get_parameter('debug_image_topic').value), 2)

    def _on_status(self, message):
        if message.state != self._last_mission_state:
            self.get_logger().info(
                f'[VISION] mission_state={message.state_text} '
                f'tree={message.current_tree_id or "-"}')
            self._last_mission_state = message.state
        self._mission_id = message.mission_id
        self._tree_id = (
            message.current_tree_id
            if message.state == MissionStatus.ARM_SPRAYING else '')
        if not self._tree_id:
            self._reported_tree_id = ''

    def _segment(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        saturation = int(self.get_parameter('saturation_min').value)
        value = int(self.get_parameter('value_min').value)
        low_1 = np.array([
            int(self.get_parameter('red_hue_low_1').value), saturation, value])
        high_1 = np.array([
            int(self.get_parameter('red_hue_high_1').value), 255, 255])
        low_2 = np.array([
            int(self.get_parameter('red_hue_low_2').value), saturation, value])
        high_2 = np.array([
            int(self.get_parameter('red_hue_high_2').value), 255, 255])
        mask = cv2.inRange(hsv, low_1, high_1) | cv2.inRange(hsv, low_2, high_2)
        size = max(1, int(self.get_parameter('morphology_kernel').value))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return hsv, mask

    def _candidates(self, hsv, mask):
        count, labels, stats, _centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        minimum = int(self.get_parameter('min_area_px').value)
        nominal = max(1, int(self.get_parameter('nominal_area_px').value))
        candidates = []
        for label in range(1, count):
            x, y, width, height, area = [int(v) for v in stats[label]]
            if area < minimum:
                continue
            component = np.zeros_like(mask)
            component[labels == label] = 255
            target_u, target_v = safest_mask_point(component)
            saturation_score = float(np.mean(hsv[:, :, 1][labels == label])) / 255.0
            area_score = min(1.0, area / nominal)
            confidence = min(0.99, 0.55 + 0.25 * saturation_score + 0.20 * area_score)
            candidates.append(Candidate(
                x, y, width, height, area,
                target_u, target_v, confidence))
        return candidates

    @staticmethod
    def _detection(candidate, header, index):
        detection = Detection2D()
        detection.header = header
        detection.id = f'disease_spot_{index}'
        detection.bbox.center.position.x = candidate.x + candidate.width / 2.0
        detection.bbox.center.position.y = candidate.y + candidate.height / 2.0
        detection.bbox.size_x = float(candidate.width)
        detection.bbox.size_y = float(candidate.height)
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = 'disease_spot'
        result.hypothesis.score = candidate.confidence
        detection.results = [result]
        return detection

    def _on_image(self, message):
        self._image_frames += 1
        first_frame = self._image_frames == 1
        if first_frame:
            self.get_logger().info(
                f'[VISION] receiving {message.width}x{message.height} images')
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'image conversion failed: {error}')
            return
        hsv, mask = self._segment(image)
        candidates = self._candidates(hsv, mask)
        if first_frame:
            self.get_logger().info(
                f'[VISION] first image processed; candidates={len(candidates)}')
        if self._tree_id and self._reported_tree_id != self._tree_id:
            self.get_logger().info(
                f'[VISION] tree={self._tree_id} mask_pixels='
                f'{int(np.count_nonzero(mask))} candidates={len(candidates)}')
            self._reported_tree_id = self._tree_id
        array = Detection2DArray()
        array.header = message.header
        array.detections = [
            self._detection(candidate, message.header, index)
            for index, candidate in enumerate(candidates)]
        self._detections.publish(array)
        if self._tree_id:
            self._publish_target(message, select_best(candidates))

        if bool(self.get_parameter('publish_debug_image').value):
            debug = image.copy()
            for candidate in candidates:
                draw_detection(
                    debug,
                    (candidate.x, candidate.y, candidate.width, candidate.height),
                    (candidate.target_u, candidate.target_v),
                    'disease_spot', candidate.confidence)
            output = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            output.header = message.header
            self._debug.publish(output)

    def _publish_target(self, image, candidate):
        target = Target2D()
        target.header = image.header
        target.mission_id = self._mission_id
        target.tree_id = self._tree_id
        target.image_width = image.width
        target.image_height = image.height
        if candidate is not None:
            target.valid = True
            target.confidence = candidate.confidence
            target.center_u = candidate.target_u
            target.center_v = candidate.target_v
            target.width = float(candidate.width)
            target.height = float(candidate.height)
        self._target.publish(target)


def main():
    rclpy.init()
    node = ColorSegmentation()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

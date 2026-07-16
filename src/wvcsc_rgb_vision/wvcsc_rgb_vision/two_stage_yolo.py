"""Two-stage C10 perception: tree detection followed by fruit segmentation."""

from dataclasses import dataclass
import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from wvcsc_interfaces.msg import MissionStatus, Target2D

from .model_utils import FRUIT_CLASS_NAMES, TREE_CLASS_NAMES, canonical_class_name, resolve_yolo_model_path


@dataclass(frozen=True)
class Instance:
    target_id: str
    class_name: str
    confidence: float
    left: float
    top: float
    right: float
    bottom: float
    aim_u: float
    aim_v: float

    @property
    def center_u(self):
        return (self.left + self.right) / 2.0

    @property
    def center_v(self):
        return (self.top + self.bottom) / 2.0

    @property
    def width(self):
        return self.right - self.left

    @property
    def height(self):
        return self.bottom - self.top

    def iou(self, other):
        left, top = max(self.left, other.left), max(self.top, other.top)
        right, bottom = min(self.right, other.right), min(self.bottom, other.bottom)
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        union = self.width * self.height + other.width * other.height - intersection
        return 0.0 if union <= 0.0 else intersection / union


def expanded_roi(left, top, right, bottom, image_width, image_height, padding):
    """Return a clipped detector ROI with a proportional border."""
    width, height = right - left, bottom - top
    pad_x, pad_y = width * padding, height * padding
    return (
        max(0, int(math.floor(left - pad_x))),
        max(0, int(math.floor(top - pad_y))),
        min(int(image_width), int(math.ceil(right + pad_x))),
        min(int(image_height), int(math.ceil(bottom + pad_y))),
    )


def safest_mask_point(points, width, height):
    """Return the furthest in-mask point, never a centroid outside the fruit."""
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.asarray(points, dtype=np.int32)], 255)
    _minimum, _maximum, _min_point, point = cv2.minMaxLoc(
        cv2.distanceTransform(mask, cv2.DIST_L2, 5))
    return float(point[0]), float(point[1])


class TwoStageYolo(Node):
    def __init__(self):
        super().__init__('wvcsc_two_stage_yolo')
        self._declare_parameters()
        self._bridge = CvBridge()
        self._mission_id = ''
        self._tree_id = ''
        self._selected_target_id = ''
        self._next_target_number = 1
        self._tracks = []
        self._tree_model, self._fruit_model = self._load_models()
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MissionStatus, '/mission/status', self._on_status, latched)
        self.create_subscription(
            String, str(self.get_parameter('selected_target_topic').value),
            self._on_selected_target, 10)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self._on_image,
            qos_profile_sensor_data)
        self._tree_pub = self.create_publisher(
            Detection2DArray, str(self.get_parameter('tree_detections_topic').value), 10)
        self._fruit_pub = self.create_publisher(
            Detection2DArray, str(self.get_parameter('fruit_detections_topic').value), 10)
        self._target_pub = self.create_publisher(
            Target2D, str(self.get_parameter('target_topic').value), 10)
        self._debug_pub = self.create_publisher(
            Image, str(self.get_parameter('debug_image_topic').value), 2)

    def _declare_parameters(self):
        values = {
            'image_topic': '/camera/camera/color/image_rect_raw',
            'selected_target_topic': '/vision/selected_target_id',
            'tree_detections_topic': '/vision/tree_detections',
            'fruit_detections_topic': '/vision/fruit_detections',
            'target_topic': '/vision/target',
            'debug_image_topic': '/vision/debug_image',
            'tree_model_path': 'wvcsc_tree_yolov8s.pt',
            'fruit_model_path': 'wvcsc_fruit_yolov8s_seg.pt',
            'tree_confidence': 0.50,
            'fruit_confidence': 0.50,
            'roi_padding': 0.10,
            'track_iou_threshold': 0.30,
            'publish_debug_image': True,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _load_models(self):
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError('Ultralytics is required for perception_mode:=yolo') from error
        paths = (
            resolve_yolo_model_path(self.get_parameter('tree_model_path').value),
            resolve_yolo_model_path(self.get_parameter('fruit_model_path').value),
        )
        missing = [path for path in paths if not Path(path).is_file()]
        if missing:
            raise FileNotFoundError(f'YOLO weight files are missing: {missing}')
        return YOLO(paths[0]), YOLO(paths[1])

    def _on_status(self, message):
        active = message.state == MissionStatus.ARM_SPRAYING
        self._mission_id = message.mission_id if active else ''
        self._tree_id = message.current_tree_id if active else ''
        if not active:
            self._selected_target_id = ''
            self._tracks = []

    def _on_selected_target(self, message):
        self._selected_target_id = message.data.strip()

    def _on_image(self, message):
        if not self._tree_id:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'image conversion failed: {error}')
            return
        tree = self._best_tree(image)
        self._tree_pub.publish(self._array(message, [] if tree is None else [tree]))
        fruits = self._fruit_instances(image, tree) if tree is not None else []
        fruits = self._assign_track_ids(fruits)
        self._fruit_pub.publish(self._array(message, fruits))
        self._publish_selected_target(message, fruits)
        if bool(self.get_parameter('publish_debug_image').value):
            self._publish_debug(message, image, tree, fruits)

    def _best_tree(self, image):
        result = self._tree_model(
            image, verbose=False, conf=float(self.get_parameter('tree_confidence').value))[0]
        instances = self._box_instances(result, TREE_CLASS_NAMES)
        if not instances:
            return None
        height, width = image.shape[:2]
        return max(
            instances,
            key=lambda item: item.confidence - 0.15 * math.hypot(
                item.center_u - width / 2.0, item.center_v - height / 2.0) / max(width, height),
        )

    def _fruit_instances(self, image, tree):
        height, width = image.shape[:2]
        x0, y0, x1, y1 = expanded_roi(
            tree.left, tree.top, tree.right, tree.bottom, width, height,
            float(self.get_parameter('roi_padding').value))
        if x1 <= x0 or y1 <= y0:
            return []
        result = self._fruit_model(
            image[y0:y1, x0:x1], verbose=False,
            conf=float(self.get_parameter('fruit_confidence').value))[0]
        return self._seg_instances(result, x0, y0, FRUIT_CLASS_NAMES)

    @staticmethod
    def _box_instances(result, class_names):
        if result.boxes is None:
            return []
        instances = []
        names = result.names
        for box in result.boxes:
            class_name = canonical_class_name(int(box.cls[0]), names)
            if class_name not in class_names.values():
                continue
            left, top, right, bottom = [float(value) for value in box.xyxy[0].tolist()]
            instances.append(Instance(
                '', class_name, float(box.conf[0]), left, top, right, bottom,
                (left + right) / 2.0, (top + bottom) / 2.0))
        return instances

    @staticmethod
    def _seg_instances(result, offset_x, offset_y, class_names):
        if result.boxes is None or result.masks is None:
            return []
        instances = []
        names = result.names
        for index, box in enumerate(result.boxes):
            class_name = canonical_class_name(int(box.cls[0]), names)
            if class_name not in class_names.values() or index >= len(result.masks.xy):
                continue
            polygon = np.asarray(result.masks.xy[index], dtype=np.float32)
            if len(polygon) < 3:
                continue
            local_width = max(1, int(math.ceil(polygon[:, 0].max())) + 1)
            local_height = max(1, int(math.ceil(polygon[:, 1].max())) + 1)
            aim_u, aim_v = safest_mask_point(polygon, local_width, local_height)
            left, top, right, bottom = [float(value) for value in box.xyxy[0].tolist()]
            instances.append(Instance(
                '', class_name, float(box.conf[0]),
                left + offset_x, top + offset_y, right + offset_x, bottom + offset_y,
                aim_u + offset_x, aim_v + offset_y))
        return instances

    def _assign_track_ids(self, instances):
        assigned = []
        threshold = float(self.get_parameter('track_iou_threshold').value)
        unmatched = list(self._tracks)
        for instance in instances:
            matches = [track for track in unmatched
                       if track.class_name == instance.class_name]
            track = max(matches, key=instance.iou, default=None)
            if track is not None and instance.iou(track) >= threshold:
                target_id = track.target_id
                unmatched.remove(track)
            else:
                target_id = f'fruit-{self._next_target_number}'
                self._next_target_number += 1
            assigned.append(Instance(target_id, instance.class_name, instance.confidence,
                                     instance.left, instance.top, instance.right,
                                     instance.bottom, instance.aim_u, instance.aim_v))
        self._tracks = assigned
        return assigned

    @staticmethod
    def _array(image, instances):
        array = Detection2DArray()
        array.header = image.header
        array.detections = [TwoStageYolo._detection(image.header, item) for item in instances]
        return array

    @staticmethod
    def _detection(header, instance):
        detection = Detection2D()
        detection.header = header
        detection.id = instance.target_id or 'tree'
        detection.bbox.center.position.x = instance.center_u
        detection.bbox.center.position.y = instance.center_v
        detection.bbox.size_x = instance.width
        detection.bbox.size_y = instance.height
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = instance.class_name
        hypothesis.hypothesis.score = instance.confidence
        detection.results = [hypothesis]
        return detection

    def _publish_selected_target(self, image, instances):
        if not self._selected_target_id:
            return
        target = next((item for item in instances
                       if item.target_id == self._selected_target_id), None)
        message = Target2D()
        message.header = image.header
        message.mission_id = self._mission_id
        message.tree_id = self._tree_id
        message.target_id = self._selected_target_id
        message.image_width = image.width
        message.image_height = image.height
        if target is not None and target.class_name == 'diseased_fruit':
            message.valid = True
            message.confidence = target.confidence
            message.center_u = target.aim_u
            message.center_v = target.aim_v
            message.width = target.width
            message.height = target.height
        self._target_pub.publish(message)

    def _publish_debug(self, image_message, image, tree, fruits):
        debug = image.copy()
        for item, color in ((tree, (0, 255, 0)),):
            if item is not None:
                cv2.rectangle(debug, (int(item.left), int(item.top)),
                              (int(item.right), int(item.bottom)), color, 2)
        for fruit in fruits:
            color = (0, 0, 255) if fruit.class_name == 'healthy_fruit' else (0, 255, 255)
            cv2.rectangle(debug, (int(fruit.left), int(fruit.top)),
                          (int(fruit.right), int(fruit.bottom)), color, 1)
            cv2.circle(debug, (round(fruit.aim_u), round(fruit.aim_v)), 3, color, -1)
        output = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        output.header = image_message.header
        self._debug_pub.publish(output)


def main():
    rclpy.init()
    node = TwoStageYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

"""C10 两阶段 YOLO 感知、病果跟踪与视觉伺服目标发布节点。

图像数据流为：整帧树木检测 -> 扩展树木 ROI -> 健康/病果分割 -> 重复实例去除 ->
跨帧 ID 关联。机械臂通过 ``/vision/selected_target_id`` 锁定一颗病果后，节点在
``target`` 模式持续发布 ``Target2D``；其 ``center_u/v`` 是分割掩膜内距离边界最远
的安全喷洒点，不是检测框中心。

所有图像坐标和尺寸单位为像素，原点在左上角，``u`` 向右、``v`` 向下。感知层
``Instance`` 只描述当前/短期跟踪实例，任务层会另外维护跨观察位的目标状态，二者
不可合并。目标关联歧义时发布无效目标并停机等待，禁止为了连续性改喷邻近病果。
"""

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import sys
import time

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

from .model_utils import (
    FRUIT_CLASS_NAMES,
    TREE_CLASS_NAMES,
    canonical_class_name,
    resolve_yolo_model_path,
    validate_yolo_model,
)


@dataclass(frozen=True)
class Instance:
    """一帧中的树或果实实例；``aim_u/v`` 是掩膜安全瞄准点。"""
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

    def distance_to(self, other):
        return math.hypot(self.center_u - other.center_u,
                          self.center_v - other.center_v)


@dataclass
class Track:
    """短期跨帧轨迹；允许少量漏检，但不会跨任务或树木复用。"""
    instance: Instance
    missed_frames: int = 0


@dataclass(frozen=True)
class TargetTemplate:
    """锁定病果的局部外观模板，用于短暂低置信度或 YOLO 空窗。"""
    patch: np.ndarray
    bbox_left: float
    bbox_top: float
    bbox_right: float
    bbox_bottom: float
    aim_u: float
    aim_v: float
    confidence: float


def capture_target_template(
        image, target, padding_ratio=0.50, min_padding_px=6.0):
    """Capture a local appearance template after a YOLO target lock."""
    height, width = image.shape[:2]
    pad_x = max(float(min_padding_px), target.width * float(padding_ratio))
    pad_y = max(float(min_padding_px), target.height * float(padding_ratio))
    left = max(0, int(math.floor(target.left - pad_x)))
    top = max(0, int(math.floor(target.top - pad_y)))
    right = min(width, int(math.ceil(target.right + pad_x)))
    bottom = min(height, int(math.ceil(target.bottom + pad_y)))
    if right - left < 3 or bottom - top < 3:
        return None
    patch = image[top:bottom, left:right].copy()
    if patch.size == 0 or float(np.std(patch)) < 1.0:
        return None
    return TargetTemplate(
        patch,
        target.left - left, target.top - top,
        target.right - left, target.bottom - top,
        target.aim_u - left, target.aim_v - top,
        target.confidence,
    )


def match_target_template(
        image, template, reference, search_radius_px=80.0,
        min_score=0.55):
    """Track a locked target through a short YOLO dropout."""
    image_height, image_width = image.shape[:2]
    template_height, template_width = template.patch.shape[:2]
    previous_left = reference.left - template.bbox_left
    previous_top = reference.top - template.bbox_top
    radius = max(0.0, float(search_radius_px))
    search_left = max(0, int(math.floor(previous_left - radius)))
    search_top = max(0, int(math.floor(previous_top - radius)))
    search_right = min(
        image_width,
        int(math.ceil(previous_left + template_width + radius)))
    search_bottom = min(
        image_height,
        int(math.ceil(previous_top + template_height + radius)))
    search = image[search_top:search_bottom, search_left:search_right]
    if (search.shape[0] < template_height or
            search.shape[1] < template_width):
        return None
    scores = cv2.matchTemplate(
        search, template.patch, cv2.TM_CCOEFF_NORMED)
    _, score, _, location = cv2.minMaxLoc(scores)
    if not math.isfinite(score) or score < float(min_score):
        return None
    patch_left = search_left + location[0]
    patch_top = search_top + location[1]
    return Instance(
        reference.target_id,
        reference.class_name,
        min(float(template.confidence), float(score)),
        patch_left + template.bbox_left,
        patch_top + template.bbox_top,
        patch_left + template.bbox_right,
        patch_top + template.bbox_bottom,
        patch_left + template.aim_u,
        patch_top + template.aim_v,
    )


def deduplicate_instances(
        instances, iou_threshold=0.35, center_distance_px=10.0,
        class_confidence_margin=0.10):
    """Collapse duplicate fruit masks and reject ambiguous class conflicts."""
    instances = list(instances)
    parents = list(range(len(instances)))

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def join(left, right):
        left_root, right_root = root(left), root(right)
        if left_root != right_root:
            parents[right_root] = left_root

    for left in range(len(instances)):
        for right in range(left + 1, len(instances)):
            if (instances[left].iou(instances[right]) >= iou_threshold or
                    instances[left].distance_to(instances[right]) <=
                    center_distance_px):
                join(left, right)

    groups = {}
    for index, instance in enumerate(instances):
        groups.setdefault(root(index), []).append(instance)

    kept = []
    for group in groups.values():
        ranked = sorted(
            group,
            key=lambda item: (
                -item.confidence, item.class_name, item.left, item.top))
        best = ranked[0]
        best_other_class = next(
            (item for item in ranked if item.class_name != best.class_name), None)
        if (best_other_class is not None and
                best.confidence - best_other_class.confidence <
                class_confidence_margin):
            continue
        kept.append(best)
    return sorted(
        kept,
        key=lambda item: (
            -item.confidence, item.class_name, item.left, item.top))


def track_matches(instances, tracks, iou_threshold, center_distance_px):
    """Return a deterministic one-to-one instance-to-track association."""
    candidates = []
    for instance_index, instance in enumerate(instances):
        for track_index, track in enumerate(tracks):
            if instance.class_name != track.instance.class_name:
                continue
            iou = instance.iou(track.instance)
            distance = instance.distance_to(track.instance)
            if iou >= iou_threshold or distance <= center_distance_px:
                # Prefer overlap matches.  Centre distance is only the fallback
                # for a detector jittering enough to lose box overlap.
                candidates.append((
                    0 if iou >= iou_threshold else 1,
                    -iou,
                    distance,
                    instance_index,
                    track_index,
                ))
    matches = {}
    used_tracks = set()
    for _kind, _iou, _distance, instance_index, track_index in sorted(candidates):
        if instance_index in matches or track_index in used_tracks:
            continue
        matches[instance_index] = track_index
        used_tracks.add(track_index)
    return matches


def reassociation_candidate(
        reference, instances, iou_threshold, center_distance_px,
        iou_margin, distance_margin_px, equivalent_aim_distance_px):
    """Return a unique selected-target replacement, or a safe failure reason."""
    scored = []
    for instance in instances:
        if instance.class_name != 'diseased_fruit':
            continue
        iou = instance.iou(reference)
        distance = instance.distance_to(reference)
        if iou >= iou_threshold or distance <= center_distance_px:
            scored.append((instance, iou, distance))
    overlap = sorted(
        (item for item in scored if item[1] >= iou_threshold),
        key=lambda item: (-item[1], item[2], item[0].target_id))
    if overlap:
        if (len(overlap) > 1 and
                overlap[0][1] - overlap[1][1] < iou_margin):
            if math.hypot(
                    overlap[0][0].aim_u - overlap[1][0].aim_u,
                    overlap[0][0].aim_v - overlap[1][0].aim_v) <= (
                        equivalent_aim_distance_px):
                return overlap[0][0], 'equivalent_reassociation'
            return None, 'ambiguous_reassociation'
        return overlap[0][0], 'none'
    nearby = sorted(scored, key=lambda item: (item[2], item[0].target_id))
    if not nearby:
        return None, 'selected_id_missing'
    if (len(nearby) > 1 and
            nearby[1][2] - nearby[0][2] < distance_margin_px):
        if math.hypot(
                nearby[0][0].aim_u - nearby[1][0].aim_u,
                nearby[0][0].aim_v - nearby[1][0].aim_v) <= (
                    equivalent_aim_distance_px):
            return nearby[0][0], 'equivalent_reassociation'
        return None, 'ambiguous_reassociation'
    return nearby[0][0], 'none'


def smoothed_target(reference, target, alpha):
    """Keep the detected geometry while damping only the spray point jitter."""
    alpha = float(alpha)
    return Instance(
        target.target_id, target.class_name, target.confidence,
        target.left, target.top, target.right, target.bottom,
        (1.0 - alpha) * reference.aim_u + alpha * target.aim_u,
        (1.0 - alpha) * reference.aim_v + alpha * target.aim_v,
    )


PERCEPTION_DEBUG_DEFAULTS = {
    'event': 'frame',
    'mission_id': '',
    'tree_id': '',
    'inference_mode': 'idle',
    'tree_found': False,
    'tree_confidence': 0.0,
    'tree_bbox_xyxy': None,
    'fruit_count': 0,
    'diseased_count': 0,
    'active_track_ids': [],
    'selected_target_id': '',
    'candidate_target_id': '',
    'selected_target_found': False,
    'target_valid': False,
    'invalid_reason': 'not_target_mode',
    'frame_latency_sec': -1.0,
}


def perception_debug_json(**values):
    """Return a stable, machine-readable perception diagnostic payload."""
    payload = dict(PERCEPTION_DEBUG_DEFAULTS)
    payload.update(values)
    return json.dumps(payload, sort_keys=True, separators=(',', ':'))


def perception_debug_due(last_time, now, rate_hz):
    """Limit steady-state debug output while allowing the first sample."""
    return (last_time is None or now - last_time >=
            1.0 / max(float(rate_hz), 1e-6) - 1e-9)


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
    """返回掩膜深部核心的稳定瞄准点，并保证结果仍位于果实内部。

    距离变换的单个最大像素对分割轮廓的一像素变化非常敏感；圆形病果还可能有
    多个等价最大值，``minMaxLoc`` 会任意选择其中一个。这里先取距离不小于最大值
    80% 的安全核心，再选择最接近核心质心的真实核心像素。该点仍有足够边界余量，
    同时避免视觉伺服因最大值在相邻像素间跳动而反复启停。
    """
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.asarray(points, dtype=np.int32)], 255)
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    maximum = float(distance.max())
    if maximum <= 0.0:
        raise ValueError('fruit mask has no interior pixel')
    core = np.argwhere(distance >= 0.80 * maximum)
    centroid = core.mean(axis=0)
    index = int(np.argmin(np.sum((core - centroid) ** 2, axis=1)))
    row, column = core[index]
    return float(column), float(row)


class TwoStageYolo(Node):
    """根据任务阶段切换推理负载，并维持单个选中病果的逻辑身份。

    ``idle/tree/fruits/target`` 模式由机械臂节点控制。图像回调串行完成推理和发布；
    MissionStatus 变化会清空树锁、轨迹和模板，避免上一棵树的 ID 污染下一棵树。
    """

    def __init__(self):
        super().__init__('wvcsc_two_stage_yolo')
        self._declare_parameters()
        self._bridge = CvBridge()
        self._mission_id = ''
        self._tree_id = ''
        self._selected_target_id = ''
        self._selected_target_reference = None
        self._selected_target_template = None
        self._inference_mode = 'idle'
        self._next_target_number = 1
        self._tracks = []
        self._locked_tree = None
        self._last_perception_debug_time = None
        self._last_perception_debug_state = None
        self._last_target_state = None
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
            String, str(self.get_parameter('inference_mode_topic').value),
            self._on_inference_mode, latched)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self._on_image,
            qos_profile_sensor_data)
        self._tree_pub = self.create_publisher(
            Detection2DArray, str(self.get_parameter('tree_detections_topic').value), 10)
        self._fruit_pub = self.create_publisher(
            Detection2DArray, str(self.get_parameter('fruit_detections_topic').value), 10)
        self._target_pub = self.create_publisher(
            Target2D, str(self.get_parameter('target_topic').value), 10)
        self._tree_visualization_pub = self.create_publisher(
            Image, str(self.get_parameter('tree_visualization_topic').value), 2)
        self._fruit_visualization_pub = self.create_publisher(
            Image, str(self.get_parameter('fruit_visualization_topic').value), 2)
        self._perception_debug_pub = self.create_publisher(
            String, str(self.get_parameter('perception_debug_topic').value), 10)

    def _declare_parameters(self):
        values = {
            'image_topic': '/camera/camera/color/image_raw',
            'selected_target_topic': '/vision/selected_target_id',
            'tree_detections_topic': '/vision/tree_detections',
            'fruit_detections_topic': '/vision/fruit_detections',
            'target_topic': '/vision/target',
            'tree_visualization_topic': '/vision/tree_debug_image',
            'fruit_visualization_topic': '/vision/fruit_debug_image',
            'perception_debug_topic': '/vision/perception_debug',
            'perception_debug_rate_hz': 5.0,
            'tree_model_path': 'wvcsc_tree_yolov8s.pt',
            'fruit_model_path': 'wvcsc_fruit_yolov8s_seg.pt',
            'inference_mode_topic': '/vision/inference_mode',
            'tree_confidence': 0.10,
            'fruit_confidence': 0.10,
            'roi_padding': 0.10,
            'track_iou_threshold': 0.20,
            'track_center_distance_px': 50.0,
            'track_max_missed_frames': 5,
            'target_reassociation_iou_margin': 0.10,
            'target_reassociation_distance_margin_px': 8.0,
            # Camera recentering can move a locked fruit farther than the
            # ordinary frame-to-frame tracker gate.  This wider gate is used
            # only while one explicit task target is selected; it is not used
            # to merge unrelated fruits into the normal track set.
            'target_reassociation_distance_px': 160.0,
            'target_equivalent_aim_distance_px': 8.0,
            'target_lock_ema_alpha': 0.20,
            'target_template_tracking_enabled': True,
            'target_template_update_min_confidence': 0.30,
            'target_template_padding_ratio': 0.50,
            'target_template_min_padding_px': 6.0,
            'target_template_search_radius_px': 80.0,
            'target_template_min_score': 0.55,
            'tree_lock_iou_threshold': 0.20,
            'publish_visualization': True,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _load_models(self):
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                f'YOLO runtime import failed with {sys.executable}: {error}. '
                'Set yolo_python_executable to the isolated WVCSC YOLO environment.'
            ) from error
        tree_path = resolve_yolo_model_path(
            self.get_parameter('tree_model_path').value)
        fruit_path = resolve_yolo_model_path(
            self.get_parameter('fruit_model_path').value)
        missing = [path for path in (tree_path, fruit_path) if not Path(path).is_file()]
        if missing:
            raise FileNotFoundError(f'YOLO weight files are missing: {missing}')
        tree_model = YOLO(tree_path)
        fruit_model = YOLO(fruit_path)
        validate_yolo_model(tree_model, 'detect', TREE_CLASS_NAMES)
        validate_yolo_model(fruit_model, 'segment', FRUIT_CLASS_NAMES)
        return tree_model, fruit_model

    def _on_status(self, message):
        active = message.state == MissionStatus.ARM_SPRAYING
        mission_id = message.mission_id if active else ''
        tree_id = message.current_tree_id if active else ''
        if (mission_id, tree_id) != (self._mission_id, self._tree_id):
            self._reset_tracking()
        self._mission_id = mission_id
        self._tree_id = tree_id
        if not active:
            self._selected_target_id = ''
            self._selected_target_reference = None
            self._selected_target_template = None

    def _on_selected_target(self, message):
        """幂等锁定逻辑目标；重复发布同一 ID 不得清空已有几何参考。"""
        selected_target_id = message.data.strip()
        if (selected_target_id and
                selected_target_id == self._selected_target_id and
                self._selected_target_reference is not None):
            return
        self._selected_target_id = selected_target_id
        self._selected_target_reference = next(
            (track.instance for track in self._tracks
             if track.instance.target_id == self._selected_target_id), None)
        self._selected_target_template = None
        self._last_target_state = None

    def _on_inference_mode(self, message):
        mode = message.data.strip()
        if mode not in {'idle', 'tree', 'fruits', 'target'}:
            self.get_logger().error(f'ignored invalid inference mode: {mode!r}')
            return
        if mode in {'idle', 'tree'}:
            self._reset_tracking()
        self._inference_mode = mode

    def _on_image(self, message):
        """执行当前模式所需的最小推理链，并发布检测、目标和诊断。"""
        if not self._tree_id or self._inference_mode == 'idle':
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'image conversion failed: {error}')
            return
        tree = self._best_tree(image)
        self._tree_pub.publish(self._array(message, [] if tree is None else [tree]))
        if bool(self.get_parameter('publish_visualization').value):
            self._publish_tree_visualization(message, image, tree)
        fruits = []
        if self._inference_mode in {'fruits', 'target'}:
            ran_fruit_inference = tree is not None
            fruits = self._assign_track_ids(
                self._fruit_instances(image, tree) if ran_fruit_inference else [])
            self._fruit_pub.publish(self._array(message, fruits))
            if ran_fruit_inference and bool(self.get_parameter('publish_visualization').value):
                self._publish_fruit_visualization(message, image, fruits)
        target = None
        invalid_reason = 'not_target_mode'
        event = 'frame'
        if self._inference_mode == 'target':
            target, invalid_reason, event = self._publish_selected_target(
                message, fruits, image)
        elif tree is None:
            invalid_reason = 'no_tree'
        elif self._inference_mode == 'fruits' and not any(
                item.class_name == 'diseased_fruit' for item in fruits):
            invalid_reason = 'no_diseased_fruit'
        self._publish_perception_debug(
            message, tree, fruits, target, invalid_reason, event)

    def _best_tree(self, image):
        result = self._tree_model(
            image, verbose=False, conf=float(self.get_parameter('tree_confidence').value))[0]
        instances = self._box_instances(result, TREE_CLASS_NAMES)
        if not instances:
            return None
        height, width = image.shape[:2]
        preferred = max(
            instances,
            key=lambda item: item.confidence - 0.15 * math.hypot(
                item.center_u - width / 2.0, item.center_v - height / 2.0) / max(width, height),
        )
        if self._locked_tree is not None:
            locked = max(instances, key=self._locked_tree.iou)
            if locked.iou(self._locked_tree) < float(
                    self.get_parameter('tree_lock_iou_threshold').value):
                return None
            preferred = locked
        self._locked_tree = preferred
        return preferred

    def _fruit_instances(self, image, tree):
        height, width = image.shape[:2]
        x0, y0, x1, y1 = expanded_roi(
            tree.left, tree.top, tree.right, tree.bottom, width, height,
            float(self.get_parameter('roi_padding').value))
        if x1 <= x0 or y1 <= y0:
            return []
        result = self._fruit_model(
            image[y0:y1, x0:x1], verbose=False,
            conf=float(self.get_parameter('fruit_confidence').value),
            iou=0.45)[0]
        return deduplicate_instances(
            self._seg_instances(result, x0, y0, FRUIT_CLASS_NAMES))

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
        """进行确定性一对一关联，避免一个历史 ID 同时分配给多个实例。"""
        assigned = []
        threshold = float(self.get_parameter('track_iou_threshold').value)
        distance = float(self.get_parameter('track_center_distance_px').value)
        matches = track_matches(instances, self._tracks, threshold, distance)
        for index, instance in enumerate(instances):
            if index in matches:
                target_id = self._tracks[matches[index]].instance.target_id
            else:
                target_id = f'fruit-{self._next_target_number}'
                self._next_target_number += 1
            assigned.append(Instance(target_id, instance.class_name, instance.confidence,
                                     instance.left, instance.top, instance.right,
                                     instance.bottom, instance.aim_u, instance.aim_v))
        retained = []
        maximum_misses = int(self.get_parameter('track_max_missed_frames').value)
        matched_tracks = set(matches.values())
        for index, track in enumerate(self._tracks):
            if index in matched_tracks:
                continue
            track.missed_frames += 1
            if track.missed_frames <= maximum_misses:
                retained.append(track)
        self._tracks = retained + [Track(instance) for instance in assigned]
        return assigned

    def _reset_tracking(self):
        self._tracks = []
        self._locked_tree = None
        self._selected_target_reference = None
        self._selected_target_template = None

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

    def _resolve_selected_target(self, instances):
        """在检测器 ID 变化时保持外部选中病果的逻辑身份。

        只有重叠或中心距离满足阈值且最佳候选与次佳候选有足够间隔时才重关联；
        歧义返回明确原因，不使用“离喷洒像素最近”之类可能选错目标的策略。
        """
        if not self._selected_target_id:
            return None, 'no_selected_target', 'target_invalid'
        reference = self._selected_target_reference
        if reference is None:
            target = next((item for item in instances
                           if item.target_id == self._selected_target_id
                           and item.class_name == 'diseased_fruit'), None)
            if target is None:
                return None, 'selected_id_missing', 'target_invalid'
            self._selected_target_reference = target
            return target, 'none', 'target_valid'
        reassociation_distance = float(self.get_parameter(
            'target_reassociation_distance_px').value)
        # When the tracker still reports the selected logical ID, keep that
        # identity if it remains geometrically plausible.  The generic
        # nearest-candidate rule is intentionally more conservative because a
        # stale ID must not steal the locked target from a nearby fruit.
        exact = next((item for item in instances
                      if item.target_id == self._selected_target_id and
                      item.class_name == 'diseased_fruit' and
                      (item.iou(reference) >= float(self.get_parameter(
                          'track_iou_threshold').value) or
                       item.distance_to(reference) <= reassociation_distance)),
                     None)
        if exact is not None:
            exact = smoothed_target(
                reference, exact,
                self.get_parameter('target_lock_ema_alpha').value)
            self._selected_target_reference = exact
            return exact, 'none', 'target_valid'
        target, reason = reassociation_candidate(
            reference,
            instances,
            float(self.get_parameter('track_iou_threshold').value),
            reassociation_distance,
            float(self.get_parameter('target_reassociation_iou_margin').value),
            float(self.get_parameter(
                'target_reassociation_distance_margin_px').value),
            float(self.get_parameter(
                'target_equivalent_aim_distance_px').value),
        )
        if target is not None:
            target = smoothed_target(
                reference, target,
                self.get_parameter('target_lock_ema_alpha').value)
            self._selected_target_reference = target
            event = ('target_valid' if target.target_id == self._selected_target_id
                     else 'target_reassociated')
            return target, 'none', event
        return None, reason, 'target_invalid'

    def _resolve_or_track_selected_target(self, image, instances):
        """优先使用 YOLO 几何关联，短时低置信度/漏检时才退化到模板跟踪。"""
        target, invalid_reason, event = self._resolve_selected_target(instances)
        tracking_enabled = (
            image is not None and
            bool(self.get_parameter(
                'target_template_tracking_enabled').value))
        update_min_confidence = (
            float(self.get_parameter(
                'target_template_update_min_confidence').value)
            if image is not None else 1.0)
        template = getattr(self, '_selected_target_template', None)
        if (
                target is not None and tracking_enabled and
                target.confidence < update_min_confidence and
                template is not None):
            tracked = match_target_template(
                image, template, self._selected_target_reference,
                self.get_parameter('target_template_search_radius_px').value,
                self.get_parameter('target_template_min_score').value)
            if tracked is not None:
                self._selected_target_reference = tracked
                return tracked, 'none', 'target_template_tracked'
        if target is not None:
            if (
                    image is not None and
                    target.confidence >= update_min_confidence):
                captured = capture_target_template(
                    image, target,
                    self.get_parameter(
                        'target_template_padding_ratio').value,
                    self.get_parameter(
                        'target_template_min_padding_px').value)
                if captured is not None:
                    if template is not None:
                        captured = replace(
                            captured,
                            confidence=max(
                                template.confidence, captured.confidence))
                    self._selected_target_template = captured
            return target, invalid_reason, event
        if (
                not tracking_enabled or
                self._selected_target_reference is None or
                template is None):
            return target, invalid_reason, event
        tracked = match_target_template(
            image,
            template,
            self._selected_target_reference,
            self.get_parameter('target_template_search_radius_px').value,
            self.get_parameter('target_template_min_score').value)
        if tracked is None:
            return target, invalid_reason, event
        self._selected_target_reference = tracked
        return tracked, 'none', 'target_template_tracked'

    def _publish_selected_target(self, image, instances, cv_image=None):
        """以原逻辑 ID 发布 Target2D；无可靠候选时仍发布 ``valid=false``。"""
        target, invalid_reason, event = self._resolve_or_track_selected_target(
            cv_image, instances)
        if not self._selected_target_id:
            return target, invalid_reason, event
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
        self._log_target_state(target, invalid_reason, event)
        return target, invalid_reason, event

    def _log_target_state(self, target, invalid_reason, event):
        """仅在目标有效性或关联事件变化时输出，避免逐帧刷屏。"""
        state = (bool(target), invalid_reason, event)
        if state == self._last_target_state:
            return
        self._last_target_state = state
        if target is None:
            self.get_logger().warn(
                f'[VISION][TARGET] invalid id={self._selected_target_id} '
                f'reason={invalid_reason}')
            return
        self.get_logger().info(
            f'[VISION][TARGET] {event} id={self._selected_target_id} '
            f'candidate={target.target_id}')

    def _publish_perception_debug(
            self, image, tree, fruits, target, invalid_reason, event):
        """发布限频 JSON，记录实际候选 ID、关联事件和图像处理延迟。"""
        now = time.monotonic()
        state = (
            self._inference_mode, tree is not None, self._selected_target_id,
            target is not None, invalid_reason, event)
        if (state == self._last_perception_debug_state and
                not perception_debug_due(
                    self._last_perception_debug_time, now,
                    self.get_parameter('perception_debug_rate_hz').value)):
            return
        stamp = image.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        frame_latency = -1.0
        if stamp_sec > 0.0:
            frame_latency = max(
                0.0, self.get_clock().now().nanoseconds * 1e-9 - stamp_sec)
        self._perception_debug_pub.publish(String(data=perception_debug_json(
            event=event,
            mission_id=self._mission_id,
            tree_id=self._tree_id,
            inference_mode=self._inference_mode,
            tree_found=tree is not None,
            tree_confidence=0.0 if tree is None else tree.confidence,
            tree_bbox_xyxy=None if tree is None else [
                round(tree.left), round(tree.top), round(tree.right), round(tree.bottom)],
            fruit_count=len(fruits),
            diseased_count=sum(
                item.class_name == 'diseased_fruit' for item in fruits),
            active_track_ids=sorted(
                track.instance.target_id for track in self._tracks),
            selected_target_id=self._selected_target_id,
            candidate_target_id='' if target is None else target.target_id,
            selected_target_found=target is not None,
            target_valid=target is not None,
            invalid_reason=invalid_reason,
            frame_latency_sec=frame_latency,
        )))
        self._last_perception_debug_time = now
        self._last_perception_debug_state = state

    @staticmethod
    def _label(instance):
        prefix = (
            f'{instance.target_id} ' if instance.target_id else '')
        return f'{prefix}{instance.class_name} {instance.confidence:.2f}'

    @staticmethod
    def _annotated_image(
            image, instances, *, draw_diseased_aim_point=False,
            selected_target_id=''):
        annotated = image.copy()
        for instance in instances:
            selected = bool(
                selected_target_id and
                instance.target_id == selected_target_id)
            color = ((255, 255, 0) if selected else
                     (0, 255, 0) if instance.class_name == 'tree' else
                     (0, 0, 255) if instance.class_name == 'healthy_fruit' else
                     (0, 255, 255))
            thickness = 4 if selected else 2
            left, top = round(instance.left), round(instance.top)
            cv2.rectangle(annotated, (left, top),
                          (round(instance.right), round(instance.bottom)),
                          color, thickness)
            cv2.putText(annotated, TwoStageYolo._label(instance),
                        (left, max(16, top - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, color, 1, cv2.LINE_AA)
            if draw_diseased_aim_point and instance.class_name == 'diseased_fruit':
                cv2.circle(annotated, (round(instance.aim_u), round(instance.aim_v)),
                           5 if selected else 3, color, -1)
        return annotated

    def _publish_visualization(self, publisher, image_message, image):
        output = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        output.header = image_message.header
        publisher.publish(output)

    def _publish_tree_visualization(self, image_message, image, tree):
        self._publish_visualization(
            self._tree_visualization_pub, image_message,
            self._annotated_image(image, [] if tree is None else [tree]))

    def _publish_fruit_visualization(self, image_message, image, fruits):
        self._publish_visualization(
            self._fruit_visualization_pub, image_message,
            self._annotated_image(
                image, fruits, draw_diseased_aim_point=True,
                selected_target_id=self._selected_target_id))


def main():
    rclpy.init()
    node = TwoStageYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()

"""实机病叶检测节点：持续推理，只向喷洒任务冻结一次最多两个目标。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from wvcsc_interfaces.msg import Target2D


@dataclass(frozen=True)
class Box:
    """一帧中的检测框，坐标均为整幅图像像素。"""

    confidence: float
    center_u: float
    center_v: float
    width: float
    height: float

    def iou(self, other: 'Box') -> float:
        left = max(self.center_u - self.width / 2.0,
                   other.center_u - other.width / 2.0)
        top = max(self.center_v - self.height / 2.0,
                  other.center_v - other.height / 2.0)
        right = min(self.center_u + self.width / 2.0,
                    other.center_u + other.width / 2.0)
        bottom = min(self.center_v + self.height / 2.0,
                     other.center_v + other.height / 2.0)
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        union = self.width * self.height + other.width * other.height - intersection
        return 0.0 if union <= 0.0 else intersection / union

    def distance(self, other: 'Box') -> float:
        return math.hypot(
            self.center_u - other.center_u, self.center_v - other.center_v)


@dataclass
class Track:
    """跨帧目标轨迹；确认后 ID 永不改变。"""

    target_id: str
    box: Box
    hits: int = 1
    missed: int = 0
    confirmed: bool = False
    first_seen: float = 0.0
    last_seen: float = 0.0


def deduplicate_boxes(boxes, iou_threshold):
    """按置信度执行一次防御性 IoU 去重。"""
    unique = []
    for box in sorted(boxes, key=lambda item: item.confidence, reverse=True):
        if any(box.iou(previous) >= iou_threshold for previous in unique):
            continue
        unique.append(box)
    return unique


class DetectNode(Node):
    """持续运行 YOLO，并冻结一次 0～2 个喷洒目标。"""

    def __init__(self):
        super().__init__('detect_node')
        self._declare_parameters()

        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            model_path = str(
                Path(get_package_share_directory('wvcsc_rgb_vision')) /
                'models' / 'best.pt')
        if not Path(model_path).is_file():
            raise FileNotFoundError(f'检测模型不存在: {model_path}')

        self.get_logger().info(f'[检测][初始化] 加载模型: {model_path}')
        self._model = YOLO(model_path)
        self._tracks = []
        self._frozen_tracks = []
        self._next_id = 1
        self._collection_started = None
        self._frozen = False
        self._selected_target_id = ''
        self._frame_count = 0
        self._last_summary_log = 0.0
        # 简化任务是一轮固定场景；该 ID 必须与内部 AlignTarget Goal 严格一致，
        # 否则视觉伺服会主动丢弃不属于当前任务的 Target2D。
        self._wait_for_collection_ready = bool(
            self.get_parameter('wait_for_collection_ready').value)
        self._collection_ready = not self._wait_for_collection_ready

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._final_pub = self.create_publisher(
            Detection2DArray,
            str(self.get_parameter('final_targets_topic').value), latched)
        self._target_pub = self.create_publisher(
            Target2D, str(self.get_parameter('tracked_target_topic').value), 10)
        self._result_pub = self.create_publisher(
            Image, str(self.get_parameter('result_image_topic').value),
            qos_profile_sensor_data)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), latched)
        self.create_subscription(
            String, str(self.get_parameter('selected_target_topic').value),
            self._on_selected_target, 10)
        self.create_subscription(
            Bool, str(self.get_parameter('collection_ready_topic').value),
            self._on_collection_ready, latched)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)

        self._publish_status(
            'WAITING_OBSERVATION'
            if self._wait_for_collection_ready else 'WAITING_IMAGE')
        self.get_logger().info(
            '[检测][就绪] 不使用 /detect/enable；收到图像后将始终执行检测')
        self.get_logger().info(
            '[检测][就绪] 收集规则: 最多 '
            f'{int(self.get_parameter("max_final_targets").value)} 个目标或 '
            f'{float(self.get_parameter("collection_timeout_sec").value):.1f} 秒')
        if self._wait_for_collection_ready:
            self.get_logger().info(
                '[检测][就绪] 持续推理并显示结果图，等待机械臂到达观察位后再收集目标')

    def _declare_parameters(self):
        values = {
            'image_topic': '/camera/color/image_raw',
            'final_targets_topic': '/detect/final_targets',
            'selected_target_topic': '/detect/selected_target_id',
            'collection_ready_topic': '/detect/collection_ready',
            'tracked_target_topic': '/detect/target',
            'result_image_topic': '/detect/result_image',
            'status_topic': '/detect/status',
            'model_path': '',
            'target_class_id': 0,
            'target_class_name': 'diseased_target',
            'confidence_threshold': 0.25,
            'collection_timeout_sec': 10.0,
            'max_final_targets': 2,
            'confirmation_frames': 3,
            'frame_dedup_iou_threshold': 0.50,
            'track_iou_threshold': 0.30,
            'track_max_missed_frames': 5,
            'reassociation_distance_px': 160.0,
            'image_width': 640,
            'image_height': 480,
            'reject_resolution_change': True,
            'debug_log_period_sec': 1.0,
            'wait_for_collection_ready': True,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)
        if int(self.get_parameter('max_final_targets').value) != 2:
            raise ValueError('简化任务要求 max_final_targets 必须为 2')
        if float(self.get_parameter('collection_timeout_sec').value) <= 0.0:
            raise ValueError('collection_timeout_sec 必须为正数')

    def _on_selected_target(self, message):
        target_id = message.data.strip()
        if target_id == self._selected_target_id:
            return
        self._selected_target_id = target_id
        self.get_logger().info(
            f'[检测][目标切换] 当前跟踪目标: {target_id or "无"}')

    def _on_collection_ready(self, message):
        """观察位信号只放行目标收集，不控制 YOLO 推理启停。"""
        if not message.data or self._collection_ready:
            return
        self._collection_ready = True
        # 机械臂移动期间看到的框不应进入最终列表，到位后从干净轨迹开始。
        self._tracks = []
        self._next_id = 1
        self._collection_started = None
        self._publish_status('WAITING_IMAGE')
        self.get_logger().info(
            '[检测][观察位就绪] 已清空移动期间轨迹，下一帧启动目标收集')

    def _on_image(self, message):
        frame = self._image_to_bgr(message)
        if frame is None:
            return
        if not self._validate_resolution(message):
            return

        # 节点始终推理；列表冻结只会关闭候选输出，不会关闭这里的模型调用。
        now = time.monotonic()
        result = self._model(
            frame, verbose=False,
            conf=float(self.get_parameter('confidence_threshold').value))[0]
        boxes = self._extract_boxes(result)
        boxes = deduplicate_boxes(
            boxes,
            float(self.get_parameter('frame_dedup_iou_threshold').value))

        # 完整模式先移动机械臂。移动期间仍推理、仍发布结果图，
        # 但不建立目标 Track、不计时、不向喷洒任务冻结目标。
        if not self._collection_ready:
            self._frame_count += 1
            self._publish_result_image(message, result, frame, now)
            self._periodic_debug(now, boxes)
            return

        if self._collection_started is None:
            self._collection_started = now
            self._publish_status('COLLECTING')
            self.get_logger().info(
                '[检测][收集开始] 观察位稳定后的第一帧到达，启动 10 秒收集窗口')

        self._update_tracks(boxes, now)
        self._frame_count += 1

        if not self._frozen:
            self._maybe_freeze(message, now)

        self._publish_selected_target(message, now)
        self._publish_result_image(message, result, frame, now)
        self._periodic_debug(now, boxes)

    def _extract_boxes(self, result):
        boxes = []
        if result.boxes is None:
            return boxes
        target_class = int(self.get_parameter('target_class_id').value)
        for item in result.boxes:
            class_id = int(item.cls[0])
            if class_id != target_class:
                continue
            left, top, right, bottom = (
                float(value) for value in item.xyxy[0].tolist())
            width = max(0.0, right - left)
            height = max(0.0, bottom - top)
            if width <= 0.0 or height <= 0.0:
                continue
            boxes.append(Box(
                float(item.conf[0]),
                (left + right) / 2.0,
                (top + bottom) / 2.0,
                width,
                height,
            ))
        return boxes

    def _update_tracks(self, boxes, now):
        """用一对一最大 IoU 更新 Track；冻结后仍更新以服务视觉伺服。"""
        threshold = float(self.get_parameter('track_iou_threshold').value)
        maximum_missed = int(
            self.get_parameter('track_max_missed_frames').value)
        unmatched_boxes = set(range(len(boxes)))
        unmatched_tracks = set(range(len(self._tracks)))
        matches = []

        scored = []
        for track_index, track in enumerate(self._tracks):
            for box_index, box in enumerate(boxes):
                score = track.box.iou(box)
                if score >= threshold:
                    scored.append((score, track_index, box_index))
        for _score, track_index, box_index in sorted(scored, reverse=True):
            if track_index not in unmatched_tracks or box_index not in unmatched_boxes:
                continue
            matches.append((track_index, box_index))
            unmatched_tracks.remove(track_index)
            unmatched_boxes.remove(box_index)

        # 机械臂运动时 IoU 可能瞬间归零。只对当前选中目标允许一次最近邻恢复，
        # 且必须是唯一最近候选，防止把另一片病叶冒充成当前喷洒目标。
        selected_index = next((
            index for index in unmatched_tracks
            if self._tracks[index].target_id == self._selected_target_id), None)
        if selected_index is not None and unmatched_boxes:
            distances = sorted(
                (self._tracks[selected_index].box.distance(boxes[index]), index)
                for index in unmatched_boxes)
            radius = float(
                self.get_parameter('reassociation_distance_px').value)
            if (distances[0][0] <= radius and
                    (len(distances) == 1 or distances[1][0] - distances[0][0] > 8.0)):
                box_index = distances[0][1]
                matches.append((selected_index, box_index))
                unmatched_tracks.remove(selected_index)
                unmatched_boxes.remove(box_index)
                self.get_logger().debug(
                    '[检测][重关联] 当前目标通过最近邻恢复 '
                    f'id={self._selected_target_id} distance={distances[0][0]:.1f}px')

        required = int(self.get_parameter('confirmation_frames').value)
        for track_index, box_index in matches:
            track = self._tracks[track_index]
            track.box = boxes[box_index]
            track.hits += 1
            track.missed = 0
            track.last_seen = now
            if not track.confirmed and track.hits >= required:
                track.confirmed = True
                self.get_logger().info(
                    '[检测][目标确认] '
                    f'id={track.target_id} confidence={track.box.confidence:.3f} '
                    f'center=({track.box.center_u:.1f},{track.box.center_v:.1f}) '
                    f'size=({track.box.width:.1f},{track.box.height:.1f})')

        for track_index in unmatched_tracks:
            self._tracks[track_index].missed += 1

        for box_index in unmatched_boxes:
            target_id = f'target-{self._next_id}'
            self._next_id += 1
            self._tracks.append(Track(
                target_id, boxes[box_index],
                first_seen=now, last_seen=now,
                confirmed=(required <= 1)))
            self.get_logger().debug(
                f'[检测][新轨迹] id={target_id} '
                f'center=({boxes[box_index].center_u:.1f},'
                f'{boxes[box_index].center_v:.1f})')

        frozen_ids = {track.target_id for track in self._frozen_tracks}
        self._tracks = [
            track for track in self._tracks
            if track.missed <= maximum_missed or track.target_id in frozen_ids
        ]

    def _maybe_freeze(self, image, now):
        confirmed = [track for track in self._tracks if track.confirmed]
        maximum = int(self.get_parameter('max_final_targets').value)
        elapsed = now - self._collection_started
        timeout = float(self.get_parameter('collection_timeout_sec').value)
        if len(confirmed) < maximum and elapsed < timeout:
            return

        reason = 'TWO_TARGETS' if len(confirmed) >= maximum else 'TIMEOUT'
        confirmed.sort(
            key=lambda track: (-track.box.confidence, track.first_seen))
        self._frozen_tracks = confirmed[:maximum]
        self._frozen = True
        final_message = self._detection_array(image, self._frozen_tracks)
        self._final_pub.publish(final_message)
        self._publish_status(f'FROZEN:{reason}:{len(self._frozen_tracks)}')
        summary = ', '.join(
            f'{track.target_id}@({track.box.center_u:.0f},'
            f'{track.box.center_v:.0f})/{track.box.confidence:.2f}'
            for track in self._frozen_tracks) or '无目标'
        self.get_logger().info(
            f'[检测][列表冻结] reason={reason} elapsed={elapsed:.2f}s '
            f'count={len(self._frozen_tracks)} targets=[{summary}]')
        self.get_logger().info(
            '[检测][列表冻结] 后续不再发布候选列表；YOLO 和结果图仍持续运行')

    def _publish_selected_target(self, image, now):
        if not self._selected_target_id:
            return
        track = next((
            item for item in self._tracks
            if item.target_id == self._selected_target_id), None)
        valid = (
            track is not None and track.missed == 0 and
            now - track.last_seen < 0.5)
        message = Target2D()
        message.header = image.header
        message.target_id = self._selected_target_id
        message.valid = bool(valid)
        message.image_width = int(image.width)
        message.image_height = int(image.height)
        if valid:
            message.confidence = float(track.box.confidence)
            message.center_u = float(track.box.center_u)
            message.center_v = float(track.box.center_v)
            message.width = float(track.box.width)
            message.height = float(track.box.height)
        self._target_pub.publish(message)

    def _publish_result_image(self, source, result, fallback_frame, now):
        try:
            annotated = result.plot()
        except Exception as error:
            self.get_logger().warn(f'[检测][绘图] YOLO 标注失败: {error}')
            annotated = fallback_frame.copy()
        if not self._collection_ready:
            state = 'WAIT OBSERVATION'
        elif self._frozen:
            state = f'FROZEN {len(self._frozen_tracks)}/2'
        else:
            remaining = max(
                0.0,
                float(self.get_parameter('collection_timeout_sec').value) -
                (now - self._collection_started))
            state = f'COLLECT {remaining:.1f}s'
        cv2.putText(
            annotated, state, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
            0.75, (0, 255, 255), 2, cv2.LINE_AA)
        if self._selected_target_id:
            cv2.putText(
                annotated, f'SELECTED {self._selected_target_id}', (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        self._result_pub.publish(self._bgr_to_image(annotated, source))

    def _periodic_debug(self, now, boxes):
        period = float(self.get_parameter('debug_log_period_sec').value)
        if now - self._last_summary_log < period:
            return
        self._last_summary_log = now
        confirmed = sum(1 for track in self._tracks if track.confirmed)
        self.get_logger().info(
            '[检测][实时] '
            f'frame={self._frame_count} boxes={len(boxes)} '
            f'tracks={len(self._tracks)} confirmed={confirmed} '
            f'frozen={len(self._frozen_tracks)} '
            f'selected={self._selected_target_id or "-"}')

    def _validate_resolution(self, message):
        expected_width = int(self.get_parameter('image_width').value)
        expected_height = int(self.get_parameter('image_height').value)
        if message.width == expected_width and message.height == expected_height:
            return True
        detail = (
            f'[检测][分辨率异常] 实际={message.width}x{message.height} '
            f'期望={expected_width}x{expected_height}')
        if bool(self.get_parameter('reject_resolution_change').value):
            self.get_logger().error(detail + '，丢弃该帧')
            return False
        self.get_logger().warn(detail + '，继续处理')
        return True

    def _detection_array(self, image, tracks):
        message = Detection2DArray()
        message.header = image.header
        message.detections = [
            self._detection(image.header, track) for track in tracks]
        return message

    def _detection(self, header, track):
        message = Detection2D()
        message.header = header
        message.id = track.target_id
        message.bbox.center.position.x = float(track.box.center_u)
        message.bbox.center.position.y = float(track.box.center_v)
        message.bbox.size_x = float(track.box.width)
        message.bbox.size_y = float(track.box.height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(
            self.get_parameter('target_class_name').value)
        hypothesis.hypothesis.score = float(track.box.confidence)
        message.results = [hypothesis]
        return message

    def _publish_status(self, state):
        self._status_pub.publish(String(data=state))

    def _image_to_bgr(self, message):
        try:
            data = np.frombuffer(message.data, dtype=np.uint8)
            if message.encoding in ('bgr8', 'rgb8'):
                image = data.reshape((message.height, message.width, 3))
                return (
                    cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                    if message.encoding == 'rgb8' else image.copy())
            if message.encoding in ('mono8', '8UC1'):
                image = data.reshape((message.height, message.width))
                return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            self.get_logger().warn(
                f'[检测][图像] 不支持的编码: {message.encoding}')
        except (ValueError, TypeError) as error:
            self.get_logger().error(f'[检测][图像] 转换失败: {error}')
        return None

    @staticmethod
    def _bgr_to_image(image, source):
        message = Image()
        message.header = source.header
        message.height = int(image.shape[0])
        message.width = int(image.shape[1])
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = int(image.shape[1] * 3)
        message.data = image.tobytes()
        return message


def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

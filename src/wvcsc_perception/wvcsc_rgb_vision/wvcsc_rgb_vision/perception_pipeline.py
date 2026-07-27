# perception_pipeline.py
"""
C10 感知流水线、病叶跟踪与视觉伺服目标发布节点。

图像数据流如下：
1. 全图病态目标分割或检测 (`disease`/`target` 模式)。
2. 对分割掩膜进行 Duplicate 去除、跨帧 ID 关联 (`assign_track_ids`)。
3. 机械臂通过 `/vision/selected_target_id` 锁定病态目标 ID 后，发布控制点给视觉伺服：
   分割使用掩膜安全点，检测使用框中心。

重要工程概念：
- 使用 `safe aim point` 代替检测框中心：通过 `cv2.distanceTransform` 计算掩膜内部
  距离边界最远的核心点，将控制点从易变的边缘拉回稳定的果实内部，极大提升了伺服稳定性。
- 任务层 ID 与感知层 ID 分离：感知层 ID 仅用于当前连续帧的短期跟踪；
  任务层 ID 由 `spray_task` 管理，跨运动重置。
- 模板匹配 (`template tracking`) 兜底：在 YOLO 由于光照或抖动短暂漏检时，
  通过 OpenCV 模板匹配延续目标的视觉追踪，防止 30 Hz 控制环因 2 帧丢失而剧烈震荡。
"""

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from wvcsc_interfaces.msg import MissionStatus, Target2D

from .disease_backend_factory import create_disease_backend
from .disease_target_backend import DiseaseTargetBackend
from .model_utils import DISEASED_TARGET_CLASS_ALIASES
from .perception_output import (
    annotated_image,
    instances_to_array,
)
from .perception_types import (
    Instance,
    Track,
    deduplicate_instances,
)
from .target_tracking import (
    TargetTemplate,
    match_target_template,
    reassociation_candidate,
    smoothed_target,
    track_matches,
    update_target_template,
)


class PerceptionPipeline(Node):
    """
    根据任务阶段切换推理负载，并维持单个选中病果的逻辑身份。
    """

    def __init__(self):
        super().__init__('wvcsc_perception_pipeline')
        self._declare_parameters()
        self._standalone_mode = bool(
            self.get_parameter('standalone_mode').value)
        self._target_class_name = str(
            self.get_parameter('target_class_name').value).strip()
        self._target_class_id = int(
            self.get_parameter('target_class_id').value)
        self._target_id_prefix = str(
            self.get_parameter('target_id_prefix').value).strip()
        self._max_diseased_targets = int(
            self.get_parameter('max_diseased_targets').value)
        if (not self._target_class_name or not self._target_id_prefix or
                self._target_class_id < 0 or
                self._max_diseased_targets < 0):
            raise ValueError('YOLO class names/prefix must be non-empty and IDs non-negative')
        self._bridge = CvBridge()
        self._mission_id = 'standalone' if self._standalone_mode else ''
        self._tree_id = 'standalone' if self._standalone_mode else ''
        self._selected_target_id = ''
        self._selected_target_reference = None
        self._selected_target_template = None
        self._inference_mode = str(self.get_parameter('inference_mode').value)
        self._next_target_number = 1
        self._tracks = []
        self._last_target_state = None
        strict_model_classes = bool(
            self.get_parameter('strict_model_classes').value)
        self._disease_model_backend = str(
            self.get_parameter('disease_model_backend').value).strip().lower()
        self._disease_backend: DiseaseTargetBackend = create_disease_backend(
            self._disease_model_backend,
            self.get_parameter('disease_model_path').value,
            self._target_class_id,
            self._configured_target_name,
            strict_model_classes=strict_model_classes)
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
        self._fruit_pub = self.create_publisher(
            Detection2DArray, str(self.get_parameter(
                'diseased_target_detections_topic').value), 10)
        self._target_pub = self.create_publisher(
            Target2D, str(self.get_parameter('target_topic').value), 10)
        self._fruit_visualization_pub = self.create_publisher(
            Image, str(self.get_parameter(
                'diseased_target_visualization_topic').value),
            qos_profile_sensor_data)
        self.get_logger().info(
            f'[YOLO][READY] standalone={self._standalone_mode} '
            f'mode={self._inference_mode} '
            f'diseased_target_backend={self._disease_model_backend} '
            f'diseased_target_model={self.get_parameter("disease_model_path").value}')

    @property
    def _configured_target_name(self):
        """Return the canonical configured target class."""
        configured = str(getattr(
            self, '_target_class_name', 'diseased_target')).strip()
        return DISEASED_TARGET_CLASS_ALIASES.get(configured, configured)

    @property
    def _configured_target_prefix(self):
        """Return the configured logical target prefix."""
        return getattr(self, '_target_id_prefix', 'target')

    def _declare_parameters(self):
        """声明并加载配置参数，与 `vision_sim.yaml` 对应。"""
        values = {
            'standalone_mode': False,
            'inference_mode': 'idle',
            'image_topic': '/camera/color/image_raw',
            'selected_target_topic': '/vision/selected_target_id',
            'diseased_target_detections_topic': (
                '/vision/diseased_target_detections'),
            'target_topic': '/vision/target',
            'diseased_target_visualization_topic': (
                '/vision/diseased_target_debug_image'),
            'disease_model_path': 'yolov8s_seg_sim.pt',
            'disease_model_backend': 'segment',
            'target_class_id': 0,
            'target_class_name': 'diseased_target',
            'target_id_prefix': 'target',
            'strict_model_classes': False,
            'inference_mode_topic': '/vision/inference_mode',
            'disease_confidence': 0.10,
            # 0 publishes every detected diseased target.
            'max_diseased_targets': 0,
            'track_iou_threshold': 0.20,
            'track_center_distance_px': 50.0,
            'track_max_missed_frames': 5,
            'target_reassociation_iou_margin': 0.10,
            'target_reassociation_distance_margin_px': 8.0,
            # 机械臂大幅度重心重定位可能使目标漂移超过 50px；
            # 专用 160px 宽关口仅允许在同一明确任务目标下恢复关联。
            'target_reassociation_distance_px': 160.0,
            'target_reassociation_require_unique_candidate': False,
            # Task-level ledger retains the original physical target.  During a
            # safe recenter, deterministic nearest association may be allowed
            # only inside the existing geometric reassociation gate.
            'target_reassociation_allow_ambiguous_nearest': False,
            'target_equivalent_aim_distance_px': 8.0,
            'target_lock_ema_alpha': 0.20,
            'target_template_tracking_enabled': True,
            'target_template_update_min_confidence': 0.30,
            'target_template_padding_ratio': 0.50,
            'target_template_min_padding_px': 6.0,
            'target_template_search_radius_px': 80.0,
            'target_template_min_score': 0.55,
            'publish_visualization': True,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _on_status(self, message):
        """监听任务状态，当任务切换时重置所有视觉跟踪状态。"""
        if self._standalone_mode:
            return
        active = message.state == MissionStatus.ARM_SPRAYING
        mission_id = message.mission_id if active else ''
        tree_id = message.current_tree_id if active else ''
        changed = (mission_id, tree_id) != (self._mission_id, self._tree_id)
        if changed:
            self._reset_tracking()
        self._mission_id = mission_id
        self._tree_id = tree_id
        if not active:
            self._selected_target_id = ''
            self._selected_target_reference = None
            self._selected_target_template = None
        if changed:
            self.get_logger().info(
                f'[YOLO][MISSION] active={active} mission={mission_id or "-"} '
                f'tree={tree_id or "-"}')

    def _on_selected_target(self, message):
        """接收来自 `spray_task` 的逻辑目标 ID 锁定指令。"""
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
        """切换推理模式 (idle/disease/target)。"""
        mode = message.data.strip()
        if mode not in {'idle', 'disease', 'target'}:
            self.get_logger().error(f'ignored invalid inference mode: {mode!r}')
            return
        if mode in {'idle', 'disease'}:
            self._reset_tracking()
        self._inference_mode = mode

    def _on_image(self, message):
        """主推理循环：根据当前模式执行最小化的推理链条。"""
        if ((not self._tree_id and not self._standalone_mode) or
                self._inference_mode == 'idle'):
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'image conversion failed: {error}')
            return
        fruits = []
        if self._inference_mode in {'disease', 'target'}:
            fruits = self._assign_track_ids(
                self._fruit_instances(image))
            fruits = self._limit_diseased_targets(fruits)
            self._fruit_pub.publish(instances_to_array(message, fruits))
            if bool(self.get_parameter('publish_visualization').value):
                self._publish_fruit_visualization(message, image, fruits)
        target = None
        invalid_reason = 'not_target_mode'
        event = 'frame'
        if self._inference_mode == 'target':
            target, invalid_reason, event = self._publish_selected_target(
                message, fruits, image)
        elif self._inference_mode == 'disease' and not any(
            item.class_name == self._configured_target_name for item in fruits):
            invalid_reason = f'no_{self._configured_target_name}'

    def _fruit_instances(self, image):
        """Run the configured disease backend on the full camera image."""
        candidates = self._disease_backend.detect(
            image,
            self.get_parameter('disease_confidence').value)
        instances = []
        for candidate in candidates:
            control_u = (candidate.center_u if candidate.control_u is None
                         else candidate.control_u)
            control_v = (candidate.center_v if candidate.control_v is None
                         else candidate.control_v)
            instances.append(Instance(
                '', candidate.class_name, candidate.confidence,
                candidate.left, candidate.top,
                candidate.right, candidate.bottom,
                control_u, control_v))
        return deduplicate_instances(instances)

    def _assign_track_ids(self, instances):
        """
        通过跨帧 IoU/距离关联，为新实例分配连贯的 `track_id`。
        """
        assigned = []
        threshold = float(self.get_parameter('track_iou_threshold').value)
        distance = float(self.get_parameter('track_center_distance_px').value)
        matches = track_matches(instances, self._tracks, threshold, distance)
        for index, instance in enumerate(instances):
            if index in matches:
                target_id = self._tracks[matches[index]].instance.target_id
            else:
                target_id = f'{self._configured_target_prefix}-{self._next_target_number}'
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

    def _limit_diseased_targets(self, instances):
        maximum = getattr(self, '_max_diseased_targets', 0)
        if maximum <= 0:
            return instances
        return sorted(instances, key=lambda item: item.confidence, reverse=True)[:maximum]

    def _reset_tracking(self):
        self._tracks = []
        self._selected_target_reference = None
        self._selected_target_template = None

    def _resolve_selected_target(self, instances):
        """
        在感知跟踪 ID 变动时，通过重关联保持外部选中的逻辑目标不变。
        
        这是重关联算法的核心网关：
        1. 优先查找感知层 ID 与逻辑 ID 相同的目标。
        2. 若找不到（发生大幅漂移），则使用宽泛关联距离 (`160px`) 寻找物理候选。
        3. 如果有多个候选且差值很小，直接判定歧义并返回 `target_invalid`，停止伺服。
        """
        if not self._selected_target_id:
            return None, 'no_selected_target', 'target_invalid'
        reference = self._selected_target_reference
        if reference is None:
            target = next((item for item in instances
                           if item.target_id == self._selected_target_id
                           and item.class_name == self._configured_target_name), None)
            if target is None:
                return None, 'selected_id_missing', 'target_invalid'
            self._selected_target_reference = target
            return target, 'none', 'target_valid'
        reassociation_distance = float(self.get_parameter(
            'target_reassociation_distance_px').value)
        exact = next((item for item in instances
                      if item.target_id == self._selected_target_id and
                      item.class_name == self._configured_target_name and
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
        try:
            require_unique_candidate = bool(self.get_parameter(
                'target_reassociation_require_unique_candidate').value)
        except (AttributeError, KeyError):
            require_unique_candidate = False
        if require_unique_candidate:
            same_class_instances = [
                item for item in instances
                if item.class_name == self._configured_target_name]
            if len(same_class_instances) > 1:
                return None, 'selected_id_missing_multiple_candidates', 'target_invalid'
        try:
            allow_ambiguous_nearest = bool(self.get_parameter(
                'target_reassociation_allow_ambiguous_nearest').value)
        except (AttributeError, KeyError):
            allow_ambiguous_nearest = False
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
            self._configured_target_name,
            allow_ambiguous_nearest,
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
        """
        优先使用 YOLO 进行几何关联，仅在当前帧没有同类 YOLO 候选时才允许
        模板跟踪兜底。

        已通过模型阈值的低置信度 YOLO 结果仍然比模板匹配更可靠。特别是在机械臂
        重心运动期间，模板外观会因视角改变而失真；用模板覆盖有效 YOLO 会让目标
        像素在两套估计之间来回跳变，导致粗对准无法收敛。
        """
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
        if target is not None:
            if (
                    image is not None and
                    target.confidence >= update_min_confidence):
                self._selected_target_template = update_target_template(
                    template,
                    image,
                    target,
                    min_confidence=update_min_confidence,
                    padding_ratio=self.get_parameter(
                        'target_template_padding_ratio').value,
                    min_padding_px=self.get_parameter(
                        'target_template_min_padding_px').value,
                )
            return target, invalid_reason, event
        # 当前帧仍有病果候选但锁定目标缺失/歧义时，禁止模板在相似果实之间
        # 猜测目标。模板只处理整个病果检测短时为空的真正漏检场景。
        same_class_instances = [
            item for item in instances
            if item.class_name == self._configured_target_name]
        if (
                not tracking_enabled or
                same_class_instances or
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
        """
        以任务层逻辑 ID 发布 Target2D 消息。
        
        如果无法找到可靠目标（歧义、漏检、模板丢失），仍然发布 `valid=false` 的 Target2D，
        让视觉伺服节点安全停止并触发超时恢复。
        """
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
        if target is not None and target.class_name == self._configured_target_name:
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
        """仅在目标状态发生变化时输出终端日志，避免高频刷屏。"""
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

    def _publish_visualization(self, publisher, image_message, image):
        output = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        output.header = image_message.header
        publisher.publish(output)

    def _publish_fruit_visualization(self, image_message, image, fruits):
        self._publish_visualization(
            self._fruit_visualization_pub, image_message,
            annotated_image(
                image, fruits, draw_diseased_aim_point=True,
                selected_target_id=self._selected_target_id,
                target_class_name=self._configured_target_name))


def main():
    rclpy.init()
    node = PerceptionPipeline()
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

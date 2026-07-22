# target_flow.py
"""
视觉目标流处理模块 (Target Flow Management)。

负责：
1. 将 ROS Detection2DArray 消息转换为任务层的 FruitTarget 实体。
2. 基于 IoU 和中心距离对目标进行防御性去重，确保单颗病果对应单一任务 ID。
3. 维护目标的跨帧可信度（确认帧计数器），防止单帧误检导致机械臂误动作。
4. 实现目标锁定、更新、丢弃和未完成（Unresolved）状态的闭环管理。
"""

import math
import time
from dataclasses import dataclass, field

from wvcsc_interfaces.action import ExecuteSpray


@dataclass(frozen=True)
class FruitTarget:
    """任务层的病果快照；生命周期跨多轮检测，不等同于感知层 Instance。

    任务层采用独立 ID 管理，即使 YOLO 因为抖动丢失了某帧，我们依然依赖
    IoU 和中心距离去将新的检测框与旧的任务实体绑定。
    """
    target_id: str
    confidence: float
    center_u: float
    center_v: float
    width: float
    height: float

    def iou(self, other):
        """计算两个矩形框的交并比 (Intersection over Union)。"""
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

    def distance_to(self, other):
        """计算两个目标中心点的欧氏距离（像素）。"""
        return math.hypot(self.center_u - other.center_u,
                          self.center_v - other.center_v)


@dataclass
class TargetAttempt:
    """同一物理病果的重试状态，防止在同一观察位重复搜索或重心。

    在观察位进行切换时，记录已经执行过重心的索引（`recentered_observation_indices`），
    确保当视觉失败后进行恢复时，系统不会死循环地在同一个物理位置反复尝试。
    """
    target: FruitTarget
    count: int = 0
    recentered_observation_indices: set = field(default_factory=set)


def detection_candidates(message, class_name, min_confidence):
    """将标准 Detection2DArray 消息转换为按置信度降序排列的任务候选列表。"""
    candidates = []
    for detection in message.detections:
        if not detection.id or not detection.results:
            continue
        hypothesis = detection.results[0].hypothesis
        if (hypothesis.class_id != class_name or
                float(hypothesis.score) < float(min_confidence)):
            continue
        bbox = detection.bbox
        candidates.append(FruitTarget(
            detection.id,
            float(hypothesis.score),
            float(bbox.center.position.x),
            float(bbox.center.position.y),
            float(bbox.size_x),
            float(bbox.size_y),
        ))
    return sorted(candidates, key=lambda item: item.confidence, reverse=True)


def deduplicate_candidates(
        candidates, iou_threshold=0.35, center_distance_px=10.0):
    """只保留同一物理果实中最强的一个任务候选（防御性去重）。"""
    unique = []
    for candidate in sorted(
            candidates, key=lambda item: item.confidence, reverse=True):
        # 检查是否与已经保留的唯一目标重合
        if any(
                candidate.iou(previous) >= iou_threshold or
                candidate.distance_to(previous) <= center_distance_px
                for previous in unique):
            continue
        unique.append(candidate)
    return unique


def spray_summary(
        detected, sprayed, unresolved, alignment_failures, recenter_attempts,
        recenter_failures, alignment_attempts):
    """生成任务执行的详细统计摘要字符串。"""
    return (
        f'detected={detected} sprayed={sprayed} unresolved={unresolved} '
        f'alignment_failures={alignment_failures} '
        f'recenter_attempts={recenter_attempts} '
        f'recenter_failures={recenter_failures} '
        f'alignment_attempts={alignment_attempts}')


def target_accounting_is_complete(detected, sprayed, unresolved):
    """校验目标统计的完整性：检测数 = 喷洒数 + 未解决数。

    防止病果静默丢失（例如视觉检测到了，但中途丢失且未被标记为未解决）。
    """
    return int(detected) == int(sprayed) + int(unresolved)


def final_spray_outcome(sprayed, unresolved, saw_disease, summary):
    """根据统计结果返回最终的 ExecuteSpray 结果码。"""
    if sprayed and unresolved:
        return ExecuteSpray.Result.PARTIAL_SUCCESS, summary
    if sprayed:
        return ExecuteSpray.Result.OK, summary
    if saw_disease:
        # 检测到了病果但全部对准/喷洒失败
        return ExecuteSpray.Result.VISION_FAILED, summary
    return (
        ExecuteSpray.Result.INSPECTED_NO_DISEASE,
        f'{summary}; tree inspected; no diseased fruit detected')


def completion_feedback_allowed(result_code):
    """判定该结果是否允许在状态机结束时发送 COMPLETED 反馈。"""
    return result_code in {
        ExecuteSpray.Result.OK,
        ExecuteSpray.Result.PARTIAL_SUCCESS,
        ExecuteSpray.Result.INSPECTED_NO_DISEASE,
    }


def target_pixel_error(center_u, center_v, desired_u, desired_v):
    """Compute error against the calibrated nozzle-axis aim pixel."""
    return float(center_u) - desired_u, float(center_v) - desired_v


def target_requires_recenter(
        center_u, center_v, desired_u, desired_v, maximum_error_px):
    """
    判断二维目标误差是否超出了允许的半径（圆范数）。

    喷洒精度是图像平面上的圆形误差预算，不是两个独立的方形阈值。使用
    欧氏范数可避免 ``(1.9, 1.4)`` px 被误当作满足 ``< 2 px`` 的情况。
    """
    error_u, error_v = target_pixel_error(
        center_u, center_v, desired_u, desired_v)
    return math.hypot(error_u, error_v) > float(maximum_error_px)


class TargetFlowMixin:
    """
    视觉目标流混入类。供 SprayTask 继承使用，提供底层的视觉目标跟踪逻辑。
    """

    # ---------- 视觉订阅回调 ----------
    def _on_tree_detections(self, message):
        """YOLO 树检测结果的回调，递增连续帧计数器。"""
        trees = detection_candidates(
            message, 'tree', self.get_parameter('tree_confidence').value)
        with self._vision_mutex:
            # 如果当前帧有有效检测，则累加帧数；否则重置计数器，要求重新累积置信度
            self._tree_frames = self._tree_frames + 1 if trees else 0

    def _on_fruit_detections(self, message):
        """YOLO 病果分割结果的回调，更新目标计数与最新快照。"""
        fruits = detection_candidates(
            message, str(self.get_parameter('target_class_name').value),
            self.get_parameter('fruit_confidence').value)
        with self._vision_mutex:
            self._fruit_frames += 1
            current = {fruit.target_id: fruit for fruit in fruits}
            # 使用计数器字典，记录每个目标连续出现的帧数
            self._fruit_counts = {
                target_id: self._fruit_counts.get(target_id, 0) + 1
                if target_id in current else 0
                for target_id in set(self._fruit_counts) | set(current)
            }
            self._fruit_latest = current

    def _on_selected_target(self, message):
        """
        /vision/target 话题的回调，处理视觉伺服（IBVS）反馈的锁定目标状态。

        这个回调充当一个复杂的“空间+时间”滤波器。
        它不仅需要确认目标 ID 匹配，还需要确认目标是否稳定地停留在
        视觉伺服工作区内。
        """
        with self._vision_mutex:
            matching_target = (
                message.target_id == self._target_confirmation_id)
            if not (
                    message.valid and matching_target and
                    message.image_width > 0 and message.image_height > 0):
                # 目标无效、不匹配或无尺寸，根据情况判断是否重置状态
                self._target_valid_frames = 0
                self._target_workspace_currently_valid = False
                now = time.monotonic()
                
                # YOLO 偶尔会漏掉单帧。短空窗内保留稳定窗口，避免 30 Hz
                # 检测链因一帧漏检永远无法满足门控；持续丢失或明确关联到
                # 另一目标时仍立即清空，防止把旧目标送入 Servo。
                short_expected_gap = (
                    not message.valid and
                    (matching_target or not message.target_id) and
                    self._target_workspace_last_seen is not None and
                    now - self._target_workspace_last_seen <=
                    self._recenter_config['post_max_gap_sec'])
                    
                if not short_expected_gap:
                    # 间隙超时或目标切换，彻底重置稳定窗口
                    self._target_confirmation_frames = 0
                    self._target_workspace_stable_since = None
                    self._target_workspace_last_seen = None
                    self._target_workspace_anchor = None
                return

            # 目标有效且匹配当前锁定目标
            self._latest_selected_target = FruitTarget(
                message.target_id,
                float(message.confidence),
                float(message.center_u),
                float(message.center_v),
                float(message.width),
                float(message.height),
            )
            self._target_valid_frames += 1

            # 检查目标是否处于可进行 IBVS 的工作空间内（像素误差小于 workspce_px）
            desired_aim = self._active_aim_pixel(
                message.image_width, message.image_height)
            reliable_in_workspace = (
                desired_aim is not None
                and
                math.isfinite(message.confidence)
                and float(message.confidence) >=
                self._recenter_config['post_min_confidence']
                and not target_requires_recenter(
                    message.center_u, message.center_v, *desired_aim,
                    self._recenter_config['workspace_px']))

            if reliable_in_workspace:
                now = time.monotonic()
                point = (float(message.center_u), float(message.center_v))
                
                # 如果目标丢失的时间超过了最大允许间隙（post_max_gap_sec），则重置窗口
                if (self._target_workspace_last_seen is not None and
                        now - self._target_workspace_last_seen >
                        self._recenter_config['post_max_gap_sec']):
                    self._target_confirmation_frames = 0
                    self._target_workspace_stable_since = None
                    self._target_workspace_anchor = None

                anchor = self._target_workspace_anchor
                if (anchor is None or math.hypot(
                        point[0] - anchor[0], point[1] - anchor[1]) >
                        self._recenter_config['post_max_drift_px']):
                    # 目标在画面中发生了显著的漂移（> 4px），重新起计稳定窗口
                    self._target_workspace_anchor = point
                    self._target_workspace_stable_since = now
                    self._target_confirmation_frames = 0
                
                self._target_workspace_last_seen = now
                self._target_workspace_currently_valid = True
                self._target_confirmation_frames += 1
            else:
                # 目标落在工作区之外或置信度不足
                self._target_confirmation_frames = 0
                self._target_workspace_stable_since = None
                self._target_workspace_last_seen = None
                self._target_workspace_anchor = None
                self._target_workspace_currently_valid = False

    # ---------- 目标等待与锁定 ----------
    def _wait_for_fruits(self, cancel_requested):
        """
        等待果实检测满足置信度与稳定性要求，返回候选列表。

        只有 `confirmation_frames` 帧连续出现，并且在 `settle` 时间后
        依然稳定，才会被认为是可靠的检测结果。
        """
        deadline = time.monotonic() + float(self.get_parameter('detection_timeout_sec').value)
        settle = float(
            self.get_parameter('fruit_collection_settle_sec').value)
        required = int(self.get_parameter('confirmation_frames').value)
        ready_since = None
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return None
            with self._vision_mutex:
                if self._fruit_frames >= required:
                    candidates = [
                        candidate for target_id, candidate in self._fruit_latest.items()
                        if self._fruit_counts.get(target_id, 0) >= required
                    ]
                    if candidates:
                        now = time.monotonic()
                        if ready_since is None:
                            ready_since = now
                        if now - ready_since >= settle:
                            return candidates
            time.sleep(0.02)
        with self._vision_mutex:
            return [] if self._fruit_frames else None

    def _reset_target_confirmation(self, target_id, *, clear_latest=True):
        """重置特定目标的确认状态，准备新一轮锁定。"""
        with self._vision_mutex:
            self._target_confirmation_id = target_id
            self._target_valid_frames = 0
            self._target_confirmation_frames = 0
            self._target_workspace_stable_since = None
            self._target_workspace_last_seen = None
            self._target_workspace_anchor = None
            self._target_workspace_currently_valid = False
            if clear_latest:
                self._latest_selected_target = None

    def _latest_target(self):
        with self._vision_mutex:
            return self._latest_selected_target

    def _wait_for_target_confirmation(
            self, target_id, cancel_requested, *, require_workspace):
        """
        确认目标在画面中持续稳定。

        如果 `require_workspace=True`，还会要求目标稳定地处于视觉伺服工作区内。
        """
        required = int(self.get_parameter('confirmation_frames').value)
        deadline = time.monotonic() + float(
            self.get_parameter('detection_timeout_sec').value)
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return False
            with self._vision_mutex:
                if target_id != self._target_confirmation_id:
                    return False
                frames = (
                    self._target_confirmation_frames if require_workspace
                    else self._target_valid_frames)
                stable_duration = (
                    0.0 if self._target_workspace_stable_since is None or
                    self._target_workspace_last_seen is None
                    else max(
                        0.0,
                        self._target_workspace_last_seen -
                        self._target_workspace_stable_since))
                stable_enough = (
                    not require_workspace or
                    (self._target_workspace_currently_valid and
                     stable_duration >= self._recenter_config['post_stable_sec']))
                if frames >= required and stable_enough:
                    return True
            time.sleep(0.02)
        return False

    def _lock_target(self, target_id, cancel_requested):
        """
        在执行任何物理运动前，捕获目标的最新状态快照作为参考。
        """
        self._reset_target_confirmation(target_id)
        self._select_target(target_id)
        self._set_inference_mode('target')
        if not self._wait_for_target_confirmation(
                target_id, cancel_requested, require_workspace=False):
            return None
        return self._latest_target()

    # ---------- 目标去重与队列管理 ----------
    def _queue(self, candidates, excluded):
        """
        根据物理距离和 IoU 过滤掉已处理或已耗尽的目标，生成待执行队列。
        """
        iou_threshold = float(self.get_parameter('processed_iou_threshold').value)
        distance_threshold = float(
            self.get_parameter('processed_center_distance_px').value)
        kept = [
            candidate for candidate in candidates
            if not any(
                candidate.iou(previous) >= iou_threshold or
                candidate.distance_to(previous) <= distance_threshold
                for previous in excluded)
        ]
        return sorted(
            deduplicate_candidates(kept),
            key=lambda item: (
                math.hypot(
                    item.center_u - float(self.get_parameter('image_width').value) / 2.0,
                    item.center_v - float(self.get_parameter('image_height').value) / 2.0),
                -item.confidence),
        )

    def _remember_targets(self, known, candidates):
        """将当前帧的新检测与已知目标进行关联，更新可能已漂移的同质目标。"""
        for candidate in candidates:
            for index, previous in enumerate(known):
                if self._same_target(candidate, previous):
                    known[index] = candidate
                    break
            else:
                known.append(candidate)

    def _replace_known_target(self, known, previous, current):
        """
        将已知列表中的旧目标替换为新目标（用于处理目标消失后的重新出现）。
        """
        for index, candidate in enumerate(known):
            if self._same_target(candidate, previous):
                known[index] = current
                known[:] = [
                    candidate for candidate_index, candidate in enumerate(known)
                    if candidate_index == index or
                    not self._same_target(candidate, current)
                ]
                return
        self._remember_targets(known, [current])

    def _pending_targets(self, known, processed, exhausted):
        """返回已知目标中既未被处理、也未被耗尽（排除）的目标。"""
        resolved = processed + exhausted
        return [
            target for target in known
            if not any(self._same_target(target, previous) for previous in resolved)
        ]

    def _attempt_for(self, candidate, attempts):
        """查找该候选是否已经存在对应的重试记录。"""
        return next((attempt for attempt in attempts if self._same_target(
            candidate, attempt.target)), None)

    def _mark_unresolved(self, target, exhausted):
        """将目标标记为未解决（Unresolved），加入排除列表。"""
        if not any(self._same_target(target, previous) for previous in exhausted):
            exhausted.append(target)

    def _same_target(self, candidate, previous):
        """
        判断两个目标是否为同一个物理目标（基于 IoU 与中心距离）。
        """
        return (
            candidate.iou(previous) >= float(
                self.get_parameter('processed_iou_threshold').value) or
            candidate.distance_to(previous) <= float(
                self.get_parameter('processed_center_distance_px').value))

    # ---------- 状态重置 ----------
    def _reset_vision(self):
        self._observation_pose = None
        self._observation_candidates = []
        self._observation_candidate_index = -1
        self._observation_distance = None
        self._tree_in_base = None
        self._camera_mount = None
        self._reset_fruit_tracking()
        self._reset_tree_tracking()

    def _reset_tree_tracking(self):
        with self._vision_mutex:
            self._tree_frames = 0

    def _reset_fruit_tracking(self):
        with self._vision_mutex:
            self._fruit_frames = 0
            self._fruit_counts = {}
            self._fruit_latest = {}
            self._target_confirmation_id = ''
            self._target_valid_frames = 0
            self._target_confirmation_frames = 0
            self._target_workspace_stable_since = None
            self._target_workspace_last_seen = None
            self._target_workspace_anchor = None
            self._target_workspace_currently_valid = False
            self._latest_selected_target = None

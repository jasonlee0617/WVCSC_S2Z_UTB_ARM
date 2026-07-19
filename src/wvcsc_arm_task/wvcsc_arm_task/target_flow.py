import math
import time
from dataclasses import dataclass, field

from wvcsc_interfaces.action import ExecuteSpray

@dataclass(frozen=True)
class FruitTarget:
    """任务层的病果快照；生命周期跨多轮检测，不等同于感知层 Instance。"""
    target_id: str
    confidence: float
    center_u: float
    center_v: float
    width: float
    height: float

    def iou(self, other):
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
        return math.hypot(self.center_u - other.center_u,
                          self.center_v - other.center_v)


@dataclass
class TargetAttempt:
    """同一物理病果的重试状态，防止在同一观察位重复搜索或重心。"""
    target: FruitTarget
    count: int = 0
    recentered_observation_indices: set = field(default_factory=set)


def detection_candidates(message, class_name, min_confidence):
    """Translate standard Detection2D messages into sorted task candidates."""
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
    """Keep only the strongest task candidate for one physical fruit."""
    unique = []
    for candidate in sorted(
            candidates, key=lambda item: item.confidence, reverse=True):
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
    return (
        f'detected={detected} sprayed={sprayed} unresolved={unresolved} '
        f'alignment_failures={alignment_failures} '
        f'recenter_attempts={recenter_attempts} '
        f'recenter_failures={recenter_failures} '
        f'alignment_attempts={alignment_attempts}')


def target_accounting_is_complete(detected, sprayed, unresolved):
    return int(detected) == int(sprayed) + int(unresolved)


def final_spray_outcome(sprayed, unresolved, saw_disease, summary):
    if sprayed and unresolved:
        return ExecuteSpray.Result.PARTIAL_SUCCESS, summary
    if sprayed:
        return ExecuteSpray.Result.OK, summary
    if saw_disease:
        return ExecuteSpray.Result.VISION_FAILED, summary
    return (
        ExecuteSpray.Result.INSPECTED_NO_DISEASE,
        f'{summary}; tree inspected; no diseased fruit detected')


def completion_feedback_allowed(result_code):
    return result_code in {
        ExecuteSpray.Result.OK,
        ExecuteSpray.Result.PARTIAL_SUCCESS,
        ExecuteSpray.Result.INSPECTED_NO_DISEASE,
    }


def target_pixel_error(
        center_u, center_v, image_width, image_height, desired_offset_u_px,
        desired_offset_v_px):
    desired_u = float(image_width) / 2.0 + float(desired_offset_u_px)
    desired_v = float(image_height) / 2.0 + float(desired_offset_v_px)
    return float(center_u) - desired_u, float(center_v) - desired_v


def target_requires_recenter(
        center_u, center_v, image_width, image_height, desired_offset_u_px,
        desired_offset_v_px, maximum_error_px):
    """Return whether the two-dimensional aim error exceeds the allowed radius.

    喷洒精度是图像平面上的圆形误差预算，不是两个独立的方形阈值。使用
    欧氏范数可避免 ``(1.9, 1.4)`` px 被误当作满足 ``< 2 px`` 的情况。
    """
    error_u, error_v = target_pixel_error(
        center_u, center_v, image_width, image_height, desired_offset_u_px,
        desired_offset_v_px)
    return math.hypot(error_u, error_v) > float(maximum_error_px)



class TargetFlowMixin:
    def _on_tree_detections(self, message):
        trees = detection_candidates(
            message, 'tree', self.get_parameter('tree_confidence').value)
        with self._vision_mutex:
            self._tree_frames = self._tree_frames + 1 if trees else 0

    def _on_fruit_detections(self, message):
        fruits = detection_candidates(
            message, 'diseased_fruit', self.get_parameter('fruit_confidence').value)
        with self._vision_mutex:
            self._fruit_frames += 1
            current = {fruit.target_id: fruit for fruit in fruits}
            self._fruit_counts = {
                target_id: self._fruit_counts.get(target_id, 0) + 1
                if target_id in current else 0
                for target_id in set(self._fruit_counts) | set(current)
            }
            self._fruit_latest = current

    def _on_selected_target(self, message):
        with self._vision_mutex:
            matching_target = (
                message.target_id == self._target_confirmation_id)
            if not (
                    message.valid and matching_target and
                    message.image_width > 0 and message.image_height > 0):
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
                    self._target_confirmation_frames = 0
                    self._target_workspace_stable_since = None
                    self._target_workspace_last_seen = None
                    self._target_workspace_anchor = None
                return
            self._latest_selected_target = FruitTarget(
                message.target_id,
                float(message.confidence),
                float(message.center_u),
                float(message.center_v),
                float(message.width),
                float(message.height),
            )
            self._target_valid_frames += 1
            reliable_in_workspace = (
                math.isfinite(message.confidence)
                and float(message.confidence) >=
                self._recenter_config['post_min_confidence']
                and not target_requires_recenter(
                    message.center_u, message.center_v, message.image_width,
                    message.image_height,
                    self._recenter_config['desired_offset_u_px'],
                    self._recenter_config['desired_offset_v_px'],
                    self._recenter_config['workspace_px']))
            if reliable_in_workspace:
                now = time.monotonic()
                point = (float(message.center_u), float(message.center_v))
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
                    # A slowly settling camera or changing mask can remain well
                    # inside the 48 px workspace while moving several pixels.
                    # Restart from the current point until one fixed-radius
                    # window remains stable for the configured duration.
                    self._target_workspace_anchor = point
                    self._target_workspace_stable_since = now
                    self._target_confirmation_frames = 0
                self._target_workspace_last_seen = now
                self._target_workspace_currently_valid = True
                self._target_confirmation_frames += 1
            else:
                self._target_confirmation_frames = 0
                self._target_workspace_stable_since = None
                self._target_workspace_last_seen = None
                self._target_workspace_anchor = None
                self._target_workspace_currently_valid = False

    def _wait_for_fruits(self, cancel_requested):
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
        """Capture a geometric reference before any camera recenter motion."""
        self._reset_target_confirmation(target_id)
        self._select_target(target_id)
        self._set_inference_mode('target')
        if not self._wait_for_target_confirmation(
                target_id, cancel_requested, require_workspace=False):
            return None
        return self._latest_target()

    def _queue(self, candidates, excluded):
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
        for candidate in candidates:
            for index, previous in enumerate(known):
                if self._same_target(candidate, previous):
                    known[index] = candidate
                    break
            else:
                known.append(candidate)

    def _replace_known_target(self, known, previous, current):
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
        resolved = processed + exhausted
        return [
            target for target in known
            if not any(self._same_target(target, previous) for previous in resolved)
        ]

    def _attempt_for(self, candidate, attempts):
        return next((attempt for attempt in attempts if self._same_target(
            candidate, attempt.target)), None)

    def _mark_unresolved(self, target, exhausted):
        if not any(self._same_target(target, previous) for previous in exhausted):
            exhausted.append(target)

    def _same_target(self, candidate, previous):
        return (
            candidate.iou(previous) >= float(
                self.get_parameter('processed_iou_threshold').value) or
            candidate.distance_to(previous) <= float(
                self.get_parameter('processed_center_distance_px').value))

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

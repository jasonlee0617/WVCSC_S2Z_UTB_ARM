# target_flow.py
"""视觉目标流处理模块。

整株发现逻辑：
1. 感知节点先按置信度过滤；SprayTask 收到结果后只做 IoU 和中心距离去重。
2. 机械臂停在一个固定观察位时，在最近 ``m`` 秒的有效推理帧内聚类同一目标。
3. 单帧置信度合格的目标只有在窗口出现率不低于配置阈值时才稳定。
4. 每个观察位最多检测 ``n`` 秒；两个稳定目标可提前结束，时间到则返回零或一个。
5. 任务层冻结最多两个目标；冻结后的实时检测只允许重关联，不允许新增目标。

跨观察位几何仅用于 IK 扫描和冻结目标恢复。joint_presets 一旦当前视角发现稳定
目标便不再换视角，因此它的初始目标确认只使用单视角图像关联。
"""

import math
import time
from dataclasses import dataclass, field, replace

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
    # The same leaf moves substantially in image pixels when the arm changes
    # observation pose.  These optional coordinates anchor it on the vertical
    # plane through the recorded tree centre, expressed as (lateral, height).
    # They are task-local geometry, not part of the ROS message contract.
    tree_lateral_m: float | None = None
    tree_height_m: float | None = None
    observation_index: int = -1

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

    def tree_plane_distance_to(self, other):
        """返回任务内树平面距离；几何不可用时返回无穷大。"""
        values = (
            self.tree_lateral_m, self.tree_height_m,
            other.tree_lateral_m, other.tree_height_m,
        )
        if not all(value is not None and math.isfinite(float(value))
                   for value in values):
            return math.inf
        return math.hypot(
            float(self.tree_lateral_m) - float(other.tree_lateral_m),
            float(self.tree_height_m) - float(other.tree_height_m))


def target_on_tree_plane(target, camera_pose, camera_model, tree_in_base,
                         observation_index):
    """为图像目标附加稳定的树平面坐标。

    C10 只有 RGB，因此采用保守几何近似：让目标像素射线与经过记录树心的垂直
    平面相交。TF 或几何不可用时保留原目标但不附加坐标，后续回退图像关联。
    """
    anchored = replace(target, observation_index=int(observation_index))
    if camera_pose is None or camera_model is None or tree_in_base is None:
        return anchored
    try:
        origin, quaternion = camera_pose
        fx, fy, cx, cy, _width, _height = camera_model
        tree_x, tree_y, tree_z = (float(value) for value in tree_in_base)
        origin = tuple(float(value) for value in origin)
        quaternion = tuple(float(value) for value in quaternion)
        values = (*origin, *quaternion, fx, fy, cx, cy,
                  tree_x, tree_y, tree_z, target.center_u, target.center_v)
        if (not all(math.isfinite(float(value)) for value in values) or
                fx <= 0.0 or fy <= 0.0):
            return anchored
        planar_range = math.hypot(tree_x, tree_y)
        if planar_range <= 1e-6:
            return anchored
        normal = (tree_x / planar_range, tree_y / planar_range, 0.0)
        local_ray = (
            (target.center_u - cx) / fx,
            (target.center_v - cy) / fy,
            1.0,
        )
        ray = _rotate_vector(local_ray, quaternion)
        denominator = sum(normal[index] * ray[index] for index in range(3))
        numerator = planar_range - sum(
            normal[index] * origin[index] for index in range(3))
        if denominator <= 1e-6 or numerator <= 0.0:
            return anchored
        depth = numerator / denominator
        point = tuple(origin[index] + depth * ray[index] for index in range(3))
        tangent = (-normal[1], normal[0], 0.0)
        relative = (
            point[0] - tree_x,
            point[1] - tree_y,
            point[2] - tree_z,
        )
        return replace(
            anchored,
            tree_lateral_m=sum(tangent[index] * relative[index]
                               for index in range(3)),
            tree_height_m=relative[2],
        )
    except (TypeError, ValueError, ZeroDivisionError):
        return anchored


def _rotate_vector(vector, quaternion):
    """不依赖 ROS 或 NumPy，使用 XYZW 四元数旋转三维向量。"""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy +
        2.0 * (xz + wy) * vz,
        2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy +
        2.0 * (yz - wx) * vz,
        2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy +
        (1.0 - 2.0 * (xx + yy)) * vz,
    )


@dataclass
class TargetAttempt:
    """同一物理病果的重试状态，防止在同一观察位重复搜索或重心。

    在观察位进行切换时，记录已经执行过重心的索引（`recentered_observation_indices`），
    确保当视觉失败后进行恢复时，系统不会死循环地在同一个物理位置反复尝试。
    """
    target: FruitTarget
    count: int = 0
    recentered_observation_indices: set = field(default_factory=set)
    # Immutable tree-level ledger snapshot.  ``target`` may be replaced by a
    # current visual observation during recovery, but this identity must never
    # drift after the first stable scan.
    ledger_target: FruitTarget | None = None


def detection_candidates(message):
    """把单类别感知结果转为任务候选并按置信度排序。"""
    candidates = []
    for detection in message.detections:
        if not detection.id or not detection.results:
            continue
        hypothesis = detection.results[0].hypothesis
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


def stable_candidates_from_frames(
        frames, confirmation_frames, iou_threshold=0.30,
        center_distance_px=18.0):
    """从短检测窗口构建不依赖跟踪 ID 的稳定目标集合。

    即使相机静止，检测跟踪 ID 也可能变化，因此任务层使用框几何聚类。同一目标
    至少出现在指定数量的不同帧才通过；逐帧一对一匹配防止重复累计。
    """
    required = max(1, int(confirmation_frames))
    clusters = []
    for frame in frames:
        assigned = set()
        for candidate in frame:
            matches = []
            for index, cluster in enumerate(clusters):
                if index in assigned:
                    continue
                previous = cluster['candidate']
                overlap = candidate.iou(previous)
                distance = candidate.distance_to(previous)
                if overlap >= iou_threshold or distance <= center_distance_px:
                    matches.append((0 if overlap >= iou_threshold else 1,
                                    -overlap, distance, index))
            if matches:
                index = min(matches)[3]
                clusters[index]['candidate'] = candidate
                clusters[index]['frames'] += 1
                assigned.add(index)
            else:
                clusters.append({'candidate': candidate, 'frames': 1})
                assigned.add(len(clusters) - 1)
    return [cluster['candidate'] for cluster in clusters
            if cluster['frames'] >= required]


def stable_candidates_by_presence(
        frames, minimum_presence_ratio, minimum_valid_frames,
        iou_threshold=0.30, center_distance_px=18.0):
    """按时间窗口出现率返回稳定的单视角目标。

    ``frames`` 必须包含窗口内的每个有效推理帧，包括没有检测框的空帧。目标在同一
    帧中最多匹配一次，避免重复框把出现率虚增。返回目标使用窗口内最近一次观测的
    几何，并保留窗口平均置信度用于排序；置信度准入已在单帧检测转换时完成。
    """
    valid_frame_count = len(frames)
    required_frames = max(1, int(minimum_valid_frames))
    if valid_frame_count < required_frames:
        return []

    clusters = []
    for frame in frames:
        assigned = set()
        for candidate in frame:
            matches = []
            for index, cluster in enumerate(clusters):
                if index in assigned:
                    continue
                previous = cluster['latest']
                overlap = candidate.iou(previous)
                distance = candidate.distance_to(previous)
                if overlap >= iou_threshold or distance <= center_distance_px:
                    matches.append((
                        0 if overlap >= iou_threshold else 1,
                        -overlap, distance, index))
            if matches:
                index = min(matches)[3]
                cluster = clusters[index]
                cluster['latest'] = candidate
                cluster['hits'] += 1
                assigned.add(index)
            else:
                clusters.append({
                    'latest': candidate,
                    'hits': 1,
                })
                assigned.add(len(clusters) - 1)

    stable = []
    ratio_threshold = float(minimum_presence_ratio)
    for cluster in clusters:
        presence_ratio = cluster['hits'] / valid_frame_count
        if presence_ratio >= ratio_threshold:
            stable.append(cluster['latest'])
    return sorted(stable, key=lambda item: item.confidence, reverse=True)


def associate_known_targets(
        known, candidates, same_target, max_cross_view_distance_px):
    """把当前检测一对一关联到冻结的树级逻辑目标。

    ``target_id`` 只是图像跟踪 ID，不代表物理身份。两侧都有树平面坐标时优先
    使用空间几何，仅在几何不可用时回退像素近邻。每个目标最多参与一次。
    """
    maximum = float(max_cross_view_distance_px)
    if not math.isfinite(maximum) or maximum <= 0.0:
        return []
    known = list(known)
    candidates = deduplicate_candidates(candidates)
    pairs = []
    used_known = set()
    used_candidates = set()

    strict = []
    for known_index, previous in enumerate(known):
        for candidate_index, candidate in enumerate(candidates):
            if same_target(candidate, previous):
                strict.append((
                    candidate.tree_plane_distance_to(previous),
                    -candidate.iou(previous),
                    candidate.distance_to(previous),
                    known_index, candidate_index))
    for _plane_distance, _negative_iou, _distance, known_index, candidate_index in sorted(strict):
        if known_index in used_known or candidate_index in used_candidates:
            continue
        pairs.append((known[known_index], candidates[candidate_index], False))
        used_known.add(known_index)
        used_candidates.add(candidate_index)

    # A view switch changes all image coordinates.  Do not compare tracker IDs
    # alone; use one-to-one nearest matching inside the explicit physical gate.
    fallback = []
    for known_index, previous in enumerate(known):
        if known_index in used_known:
            continue
        for candidate_index, candidate in enumerate(candidates):
            if candidate_index in used_candidates:
                continue
            # When both views have tree-plane anchors, a failed spatial match
            # must not be rescued by the former 320 px nearest-neighbour rule:
            # that rule can exchange two nearby leaves after a view change.
            if math.isfinite(candidate.tree_plane_distance_to(previous)):
                continue
            distance = candidate.distance_to(previous)
            if distance <= maximum:
                fallback.append((distance, -candidate.confidence,
                                 known_index, candidate_index))
    for _distance, _confidence, known_index, candidate_index in sorted(fallback):
        if known_index in used_known or candidate_index in used_candidates:
            continue
        pairs.append((known[known_index], candidates[candidate_index], True))
        used_known.add(known_index)
        used_candidates.add(candidate_index)
    return pairs


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


def target_accounting(known, processed, exhausted, same_target):
    """传递合并恢复快照后统计逻辑目标的处理状态。"""
    items = (
        [(target, 'known') for target in known] +
        [(target, 'processed') for target in processed] +
        [(target, 'exhausted') for target in exhausted])
    parents = list(range(len(items)))

    def find(index):
        """查找并压缩并查集中的目标根节点。"""
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def join(left, right):
        """合并两个属于同一物理目标的并查集分组。"""
        left, right = find(left), find(right)
        if left != right:
            parents[right] = left

    for left, (candidate, _state) in enumerate(items):
        for right in range(left):
            if same_target(candidate, items[right][0]):
                join(left, right)

    groups = {}
    for index, item in enumerate(items):
        groups.setdefault(find(index), []).append(item)

    sprayed = 0
    unresolved = 0
    pending = []
    for snapshots in groups.values():
        states = {state for _target, state in snapshots}
        if 'processed' in states:
            sprayed += 1
        elif 'exhausted' in states:
            unresolved += 1
        else:
            pending.append(snapshots[0][0])
    return len(groups), sprayed, unresolved, pending


def limit_targets_per_tree(known, candidates, maximum, same_target):
    """冻结初始高置信度目标，同时允许已冻结目标被重新检测。"""
    candidates = deduplicate_candidates(candidates)
    if maximum <= 0:
        return candidates

    accepted = []
    new_targets = 0
    for candidate in candidates:
        if any(same_target(candidate, previous) for previous in known):
            accepted.append(candidate)
        elif len(known) + new_targets < maximum:
            accepted.append(candidate)
            new_targets += 1
    return accepted


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
    """计算目标中心相对标定喷嘴轴线像素的二维误差。"""
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
    def _on_fruit_detections(self, message):
        """接收感知层已完成置信度过滤的检测结果并保留短窗口快照。"""
        fruits = deduplicate_candidates(detection_candidates(message))
        with self._vision_mutex:
            self._fruit_frames += 1
            self._fruit_history.append((time.monotonic(), list(fruits)))

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

            # 重心后才允许交给 IBVS；这里使用 Servo 的真实入口阈值，而不是
            # 已删除的粗对准固定工作区阈值。
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
                    self._recenter_config['servo_entry_px']))

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
        """收集稳定病害目标，并按当前阶段选择发现或复检策略。

        初次发现使用单视角总时间和滑动出现率；冻结后的复检沿用短稳定窗口，避免
        每喷一个目标都重新等待完整的观察位发现时长。
        """
        if getattr(self, '_target_discovery_active', False):
            return self._wait_for_discovery_targets(cancel_requested)

        deadline = time.monotonic() + float(self.get_parameter('detection_timeout_sec').value)
        settle = float(
            self.get_parameter('fruit_collection_settle_sec').value)
        required = int(self.get_parameter('confirmation_frames').value)
        collection_started_at = None
        result = None
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return None
            with self._vision_mutex:
                if collection_started_at is None:
                    collection_started_at = next(
                        (stamp for stamp, candidates in self._fruit_history
                         if candidates), None)
                if (collection_started_at is not None and
                        time.monotonic() - collection_started_at >= settle):
                    frames = [
                        candidates for stamp, candidates in self._fruit_history
                        if collection_started_at <= stamp <=
                        collection_started_at + settle
                    ]
                    result = stable_candidates_from_frames(
                        frames, required,
                        float(self.get_parameter('processed_iou_threshold').value),
                        float(self.get_parameter(
                            'processed_center_distance_px').value))
            if result is not None:
                return self._anchor_targets_to_tree_plane(result)
            time.sleep(0.02)
        with self._vision_mutex:
            result = [] if self._fruit_frames else None
        return (None if result is None else
                self._anchor_targets_to_tree_plane(result))

    def _wait_for_discovery_targets(self, cancel_requested):
        """在一个静止观察位内检测至两个稳定目标或 ``n`` 秒到期。

        每次判断只使用最近 ``m`` 秒的有效推理帧。达到两个稳定目标时提前返回；
        总时间到期时返回当前稳定的零或一个目标。整个窗口没有任何有效推理帧返回
        ``None``，让上层区分“没有病害”与“视觉链路失效”。
        """
        started = time.monotonic()
        duration = float(
            self.get_parameter('view_detection_duration_sec').value)
        presence_window = float(
            self.get_parameter('target_presence_window_sec').value)
        presence_ratio = float(
            self.get_parameter('target_presence_ratio').value)
        minimum_frames = int(
            self.get_parameter('target_presence_min_frames').value)
        maximum = int(self.get_parameter('max_targets_per_tree').value)
        deadline = started + duration
        latest_stable = []

        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return None
            now = time.monotonic()
            window_start = max(started, now - presence_window)
            with self._vision_mutex:
                frames = [
                    candidates for stamp, candidates in self._fruit_history
                    if window_start <= stamp <= now
                ]
            if now - started >= presence_window:
                latest_stable = stable_candidates_by_presence(
                    frames,
                    presence_ratio,
                    minimum_frames,
                    float(self.get_parameter(
                        'processed_iou_threshold').value),
                    float(self.get_parameter(
                        'processed_center_distance_px').value),
                )
                if maximum > 0 and len(latest_stable) >= maximum:
                    return self._anchor_targets_to_tree_plane(
                        latest_stable[:maximum])
            time.sleep(0.02)

        with self._vision_mutex:
            received_frames = any(
                stamp >= started for stamp, _candidates in self._fruit_history)
        if not received_frames:
            return None
        if maximum > 0:
            latest_stable = latest_stable[:maximum]
        return self._anchor_targets_to_tree_plane(latest_stable)

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
        """线程安全地返回当前选中目标的最新快照。"""
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
        kept = [
            candidate for candidate in candidates
            if not any(
                self._same_target(candidate, previous)
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

    def _associate_known_targets(self, known, candidates):
        """把新视角检测关联到不可扩张的树级目标账本。

        感知层负责逐帧跟踪，任务层防止相机移动和临时 ID 变化让待处理物理目标
        从账本中消失。
        """
        maximum = float(self.get_parameter(
            'cross_view_reassociation_max_distance_px').value)
        return associate_known_targets(
            known, candidates, self._same_target, maximum)

    def _remember_targets(self, known, candidates):
        """将当前帧的新检测与已知目标进行关联，更新可能已漂移的同质目标。"""
        for candidate in candidates:
            for index, previous in enumerate(known):
                if self._same_target(candidate, previous):
                    known[index] = candidate
                    break
            else:
                known.append(candidate)

    def _merge_discovered_targets(self, known, candidates):
        """合并 IK 扫描视角，并为同一目标保留最清晰的观测。"""
        for candidate in candidates:
            matching = next((index for index, previous in enumerate(known)
                             if self._same_target(candidate, previous)), None)
            if matching is None:
                known.append(candidate)
            elif candidate.confidence > known[matching].confidence:
                known[matching] = candidate

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
            candidate, getattr(attempt, 'ledger_target', None) or attempt.target)), None)

    def _mark_unresolved(self, target, exhausted):
        """将目标标记为未解决（Unresolved），加入排除列表。"""
        if not any(self._same_target(target, previous) for previous in exhausted):
            exhausted.append(target)

    def _same_target(self, candidate, previous):
        """
        判断两个目标是否为同一个物理目标（基于 IoU 与中心距离）。
        """
        try:
            plane_gate = float(self.get_parameter(
                'cross_view_target_distance_m').value)
        except (AttributeError, KeyError, TypeError, ValueError):
            plane_gate = 0.0
        if (plane_gate > 0.0 and
                candidate.tree_plane_distance_to(previous) <= plane_gate):
            return True
        return (
            candidate.iou(previous) >= float(
                self.get_parameter('processed_iou_threshold').value) or
            candidate.distance_to(previous) <= float(
                self.get_parameter('processed_center_distance_px').value))

    def _anchor_targets_to_tree_plane(self, candidates):
        """机械臂静止后为稳定候选附加树平面空间坐标。"""
        try:
            camera_pose = self._current_camera_pose()
        except (AttributeError, TypeError, ValueError):
            camera_pose = None
        with self._state_mutex:
            camera_model = self._camera_model
        return [target_on_tree_plane(
            candidate, camera_pose, camera_model, self._tree_in_base,
            self._observation_candidate_index)
            for candidate in candidates]

    # ---------- 状态重置 ----------
    def _reset_vision(self):
        """清空本次整株任务的观察位、几何和目标跟踪状态。"""
        self._observation_pose = None
        self._observation_candidates = []
        self._observation_candidate_index = -1
        self._observation_distance = None
        self._tree_in_base = None
        self._camera_mount = None
        self._reset_fruit_tracking()

    def _reset_fruit_tracking(self):
        """清空当前观察位的检测历史和选中目标确认状态。"""
        with self._vision_mutex:
            self._fruit_frames = 0
            self._fruit_history = []
            self._target_confirmation_id = ''
            self._target_valid_frames = 0
            self._target_confirmation_frames = 0
            self._target_workspace_stable_since = None
            self._target_workspace_last_seen = None
            self._target_workspace_anchor = None
            self._target_workspace_currently_valid = False
            self._latest_selected_target = None

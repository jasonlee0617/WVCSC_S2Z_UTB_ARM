# 中文说明：植株级病态目标账本与几何辅助模块。
# 它不依赖 ROS，负责跨观察位保持目标身份、去重和“每个目标最多喷洒一次”的统计契约。
"""Pure tree-level disease-target ledger and geometry helpers.

This module deliberately has no ROS imports.  It owns the task-local target
snapshot, cross-view association and accounting rules used to prevent a leaf
from being sprayed twice when a camera observation changes.
"""

import math
from dataclasses import dataclass, field, replace


@dataclass(frozen=True)
class FruitTarget:
    """Task-level disease-target snapshot, independent of a detector tracker ID."""

    target_id: str
    confidence: float
    center_u: float
    center_v: float
    width: float
    height: float
    # Optional task-local coordinates on the vertical plane through the tree.
    # They are not part of the ROS message contract.
    tree_lateral_m: float | None = None
    tree_height_m: float | None = None
    observation_index: int = -1

    def iou(self, other):
        """Return the intersection-over-union of two image-space boxes."""
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
        """Return the Euclidean distance between image-space centers."""
        return math.hypot(self.center_u - other.center_u,
                          self.center_v - other.center_v)

    def tree_plane_distance_to(self, other):
        """Return tree-plane distance, or ``inf`` when no anchor is available."""
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
    """Attach a stable vertical-tree-plane anchor to one image-space target."""
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
    """Rotate a vector by an XYZW quaternion without ROS or NumPy."""
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
    """Retry state for one physical target across observation changes."""

    target: FruitTarget
    count: int = 0
    recentered_observation_indices: set = field(default_factory=set)
    # ``target`` may be replaced by the current detection; the ledger snapshot
    # never changes after the first stable scan.
    ledger_target: FruitTarget | None = None


def detection_candidates(message, class_name, min_confidence):
    """Convert a Detection2DArray-like message into confidence-sorted targets."""
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
    """Keep only the strongest candidate for each overlapping target."""
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


def stable_candidates_from_frames(
        frames, confirmation_frames, iou_threshold=0.30,
        center_distance_px=18.0):
    """Build an ID-independent logical target set from a short scan window."""
    required = max(1, int(confirmation_frames))
    clusters = []
    for frame in frames:
        assigned = set()
        for candidate in deduplicate_candidates(frame):
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
    """Return targets stable over a window of all valid inference frames.

    ``frames`` includes empty frames.  A target contributes at most once to a
    frame, so duplicate detector boxes cannot inflate its appearance rate.
    The latest valid geometry is returned because it is the correct snapshot
    to aim from; confidence is already filtered per frame by the ROS layer.
    """
    valid_frame_count = len(frames)
    if valid_frame_count < max(1, int(minimum_valid_frames)):
        return []

    clusters = []
    for frame in frames:
        assigned = set()
        for candidate in deduplicate_candidates(
                frame, iou_threshold, center_distance_px):
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
                clusters[index]['latest'] = candidate
                clusters[index]['hits'] += 1
                assigned.add(index)
            else:
                clusters.append({'latest': candidate, 'hits': 1})
                assigned.add(len(clusters) - 1)

    threshold = float(minimum_presence_ratio)
    return sorted(
        (cluster['latest'] for cluster in clusters
         if cluster['hits'] / valid_frame_count >= threshold),
        key=lambda item: item.confidence,
        reverse=True)


def associate_known_targets(
        known, candidates, same_target, max_cross_view_distance_px):
    """Associate detections one-to-one with immutable logical targets."""
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

    fallback = []
    for known_index, previous in enumerate(known):
        if known_index in used_known:
            continue
        for candidate_index, candidate in enumerate(candidates):
            if candidate_index in used_candidates:
                continue
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
    """Return the existing ExecuteSpray summary string."""
    return (
        f'detected={detected} sprayed={sprayed} unresolved={unresolved} '
        f'alignment_failures={alignment_failures} '
        f'recenter_attempts={recenter_attempts} '
        f'recenter_failures={recenter_failures} '
        f'alignment_attempts={alignment_attempts}')


def target_accounting_is_complete(detected, sprayed, unresolved):
    """Return whether every detected logical target is resolved exactly once."""
    return int(detected) == int(sprayed) + int(unresolved)


def target_accounting(known, processed, exhausted, same_target):
    """Count logical targets after transitively merging recovery snapshots."""
    items = (
        [(target, 'known') for target in known] +
        [(target, 'processed') for target in processed] +
        [(target, 'exhausted') for target in exhausted])
    parents = list(range(len(items)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def join(left, right):
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
    """Keep initial high-confidence targets while allowing re-detection."""
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


def target_pixel_error(center_u, center_v, desired_u, desired_v):
    """Compute error against the calibrated nozzle-axis aim pixel."""
    return float(center_u) - desired_u, float(center_v) - desired_v


def target_requires_recenter(
        center_u, center_v, desired_u, desired_v, maximum_error_px):
    """Return whether the target lies outside the circular aim tolerance."""
    error_u, error_v = target_pixel_error(
        center_u, center_v, desired_u, desired_v)
    return math.hypot(error_u, error_v) > float(maximum_error_px)

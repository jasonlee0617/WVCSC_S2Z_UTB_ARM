"""Pure data structures and geometry shared by the perception pipeline."""

from dataclasses import dataclass
import math

from .model_utils import DISEASED_TARGET_CLASS_ALIASES


@dataclass(frozen=True)
class Instance:
    """A detected diseased target, expressed in image pixels."""

    target_id: str
    class_name: str
    confidence: float
    left: float
    top: float
    right: float
    bottom: float
    aim_u: float
    aim_v: float

    def __post_init__(self):
        object.__setattr__(
            self,
            'class_name',
            DISEASED_TARGET_CLASS_ALIASES.get(
                str(self.class_name), str(self.class_name)),
        )

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
        return math.hypot(
            self.center_u - other.center_u, self.center_v - other.center_v)


@dataclass(frozen=True)
class DiseaseTarget:
    """One disease-model result in the full-image coordinate system.

    ``control_u/v`` are optional.  Segment backends provide a mask-safe
    control point; detect backends intentionally leave them unset so that the
    pipeline uses the bounding-box centre.
    """

    class_name: str
    confidence: float
    left: float
    top: float
    right: float
    bottom: float
    control_u: float | None = None
    control_v: float | None = None

    def __post_init__(self):
        if (self.control_u is None) != (self.control_v is None):
            raise ValueError('disease control point must contain both u and v')
        object.__setattr__(
            self,
            'class_name',
            DISEASED_TARGET_CLASS_ALIASES.get(
                str(self.class_name), str(self.class_name)),
        )

    @property
    def center_u(self):
        return (self.left + self.right) / 2.0

    @property
    def center_v(self):
        return (self.top + self.bottom) / 2.0


@dataclass
class Track:
    """Short-lived cross-frame identity state."""

    instance: Instance
    missed_frames: int = 0


def deduplicate_instances(
        instances, iou_threshold=0.35, center_distance_px=10.0):
    """Keep the highest-confidence member of each duplicate instance group."""
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
        kept.append(sorted(
            group,
            key=lambda item: (
                -item.confidence, item.class_name, item.left, item.top),
        )[0])
    return sorted(
        kept,
        key=lambda item: (-item.confidence, item.class_name, item.left, item.top),
    )

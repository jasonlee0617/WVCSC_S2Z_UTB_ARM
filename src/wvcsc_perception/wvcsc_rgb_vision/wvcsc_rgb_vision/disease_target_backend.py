# 中文说明：病态目标后端的最小协议。
# 该协议隔离模型推理与 ROS 流水线，允许同事替换 detect/segment 实现而不改下游话题。
"""Minimal model-facing contract for disease-target inference backends."""

from typing import Protocol

import numpy as np

from .perception_types import DiseaseTarget


class DiseaseTargetBackend(Protocol):
    """Infer disease targets in full-camera image coordinates.

    Backends return boxes and an optional full-image control point.  The ROS
    pipeline owns tracking, selected-target recovery, and publishing.
    """

    def detect(self, image: np.ndarray, confidence: float) -> list[DiseaseTarget]:
        """Return configured disease targets in ``image`` coordinates."""

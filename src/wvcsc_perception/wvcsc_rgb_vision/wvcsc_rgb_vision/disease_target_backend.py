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

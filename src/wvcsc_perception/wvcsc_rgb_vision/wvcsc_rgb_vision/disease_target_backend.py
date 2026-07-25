"""Minimal model-facing contract for disease-target inference backends."""

from typing import Protocol

import numpy as np

from .perception_types import DiseaseTarget


class DiseaseTargetBackend(Protocol):
    """Infer disease targets in a tree ROI.

    Backends only see ROI-local image coordinates.  They return bounding boxes
    and may attach a ROI-local control point.  The ROS pipeline owns ROI
    extraction, full-image coordinate restoration, tracking, and publishing.
    """

    def detect(self, roi_image: np.ndarray, confidence: float) -> list[DiseaseTarget]:
        """Return configured disease targets in ``roi_image`` coordinates."""


"""YOLO segmentation backend for disease targets."""

import math

import cv2
import numpy as np

from .model_utils import (
    CANONICAL_DISEASE_TARGET_CLASS_NAME,
    load_yolo_model,
)
from .perception_types import DiseaseTarget


def safest_mask_point(points, width, height):
    """Return a stable point furthest from this segment mask boundary."""
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(mask, [np.asarray(points, dtype=np.int32)], 255)
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    maximum = float(distance.max())
    if maximum <= 0.0:
        raise ValueError('disease mask has no interior pixel')
    core = np.argwhere(distance >= 0.80 * maximum)
    centroid = core.mean(axis=0)
    row, column = core[int(np.argmin(np.sum((core - centroid) ** 2, axis=1)))]
    return float(column), float(row)


class DiseaseSegmenter:
    """Run a fixed ``segment`` model and return full-image safe control points."""

    def __init__(
            self, model_path, target_class_id, model_target_class_name,
            *, strict_model_classes=False):
        self._target_class_id = int(target_class_id)
        self._model_target_class_name = str(model_target_class_name)
        self._model = self._load_model(model_path, strict_model_classes)

    def _load_model(self, model_path, strict_model_classes):
        return load_yolo_model(
            model_path, 'segment',
            {self._target_class_id: self._model_target_class_name},
            exact_names=strict_model_classes)

    def detect(self, image, confidence):
        """Return target boxes and mask-safe points in image coordinates."""
        result = self._model(
            image, verbose=False, conf=float(confidence), iou=0.45)[0]
        return self._instances(result)

    def _instances(self, result):
        if result.boxes is None or result.masks is None:
            return []
        instances = []
        for index, box in enumerate(result.boxes):
            if index >= len(result.masks.xy):
                continue
            polygon = np.asarray(result.masks.xy[index], dtype=np.float32)
            if len(polygon) < 3:
                continue
            local_width = max(1, int(math.ceil(polygon[:, 0].max())) + 1)
            local_height = max(1, int(math.ceil(polygon[:, 1].max())) + 1)
            control_u, control_v = safest_mask_point(
                polygon, local_width, local_height)
            left, top, right, bottom = [
                float(value) for value in box.xyxy[0].tolist()]
            instances.append(DiseaseTarget(
                CANONICAL_DISEASE_TARGET_CLASS_NAME, float(box.conf[0]),
                left, top, right, bottom, control_u, control_v))
        return instances

"""YOLO segmentation backend for disease targets."""

import math
from pathlib import Path
import sys

import numpy as np

from .model_utils import resolve_yolo_model_path, validate_yolo_model
from .perception_types import DiseaseTarget, safest_mask_point


class DiseaseSegmenter:
    """Run a fixed ``segment`` model and return ROI-local safe control points."""

    def __init__(
            self, model_path, target_class_id, target_class_name,
            *, strict_model_classes=False):
        self._target_class_id = int(target_class_id)
        self._target_class_name = str(target_class_name)
        self._model = self._load_model(model_path, strict_model_classes)

    def _load_model(self, model_path, strict_model_classes):
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                f'YOLO runtime import failed with {sys.executable}: {error}. '
                'Set yolo_python_executable to the isolated WVCSC YOLO environment.'
            ) from error
        resolved_path = resolve_yolo_model_path(model_path)
        if not Path(resolved_path).is_file():
            raise FileNotFoundError(f'YOLO weight file is missing: {resolved_path}')
        model = YOLO(resolved_path)
        validate_yolo_model(
            model, 'segment',
            {self._target_class_id: self._target_class_name},
            exact_names=strict_model_classes)
        return model

    def detect(self, roi_image, confidence):
        """Return target boxes and mask-safe points in ROI-local coordinates."""
        result = self._model(
            roi_image, verbose=False, conf=float(confidence), iou=0.45)[0]
        return self._instances(result)

    def _instances(self, result):
        if result.boxes is None or result.masks is None:
            return []
        instances = []
        for index, box in enumerate(result.boxes):
            if int(box.cls[0]) != self._target_class_id or index >= len(result.masks.xy):
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
                self._target_class_name, float(box.conf[0]),
                left, top, right, bottom, control_u, control_v))
        return instances

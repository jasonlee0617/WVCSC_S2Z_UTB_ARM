"""
YOLO detection backend for disease targets.

“框中心”确实落在可喷洒区域，避免框中心落在叶片间隙或背景上
"""

from .model_utils import (
    CANONICAL_DISEASE_TARGET_CLASS_NAME,
    load_yolo_model,
)
from .perception_types import DiseaseTarget


class DiseaseDetector:
    """Run a fixed ``detect`` model and return full-image bounding boxes only."""

    def __init__(
            self, model_path, target_class_id, model_target_class_name,
            *, strict_model_classes=False):
        self._target_class_id = int(target_class_id)
        self._model_target_class_name = str(model_target_class_name)
        self._model = self._load_model(model_path, strict_model_classes)

    def _load_model(self, model_path, strict_model_classes):
        return load_yolo_model(
            model_path, 'detect',
            {self._target_class_id: self._model_target_class_name},
            exact_names=strict_model_classes)

    def detect(self, image, confidence):
        """Return configured disease boxes without a model-provided aim point."""
        result = self._model(image, verbose=False, conf=float(confidence))[0]
        if result.boxes is None:
            return []
        instances = []
        for box in result.boxes:
            left, top, right, bottom = [
                float(value) for value in box.xyxy[0].tolist()]
            instances.append(DiseaseTarget(
                CANONICAL_DISEASE_TARGET_CLASS_NAME, float(box.conf[0]),
                left, top, right, bottom))
        return instances

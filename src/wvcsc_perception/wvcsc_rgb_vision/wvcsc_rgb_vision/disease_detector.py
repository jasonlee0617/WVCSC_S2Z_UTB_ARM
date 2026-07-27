"""YOLO detection backend for disease targets."""

from .model_utils import load_yolo_model
from .perception_types import DiseaseTarget


class DiseaseDetector:
    """Run a fixed ``detect`` model and return full-image bounding boxes only."""

    def __init__(
            self, model_path, target_class_id, target_class_name,
            *, strict_model_classes=False):
        self._target_class_id = int(target_class_id)
        self._target_class_name = str(target_class_name)
        self._model = self._load_model(model_path, strict_model_classes)

    def _load_model(self, model_path, strict_model_classes):
        return load_yolo_model(
            model_path, 'detect',
            {self._target_class_id: self._target_class_name},
            exact_names=strict_model_classes)

    def detect(self, image, confidence):
        """Return configured disease boxes without a model-provided aim point."""
        result = self._model(image, verbose=False, conf=float(confidence))[0]
        if result.boxes is None:
            return []
        instances = []
        for box in result.boxes:
            if int(box.cls[0]) != self._target_class_id:
                continue
            left, top, right, bottom = [
                float(value) for value in box.xyxy[0].tolist()]
            instances.append(DiseaseTarget(
                self._target_class_name, float(box.conf[0]),
                left, top, right, bottom))
        return instances

"""YOLO detection backend for disease targets."""

from pathlib import Path
import sys

from .model_utils import resolve_yolo_model_path, validate_yolo_model
from .perception_types import DiseaseTarget


class DiseaseDetector:
    """Run a fixed ``detect`` model and return ROI-local bounding boxes only."""

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
            model, 'detect',
            {self._target_class_id: self._target_class_name},
            exact_names=strict_model_classes)
        return model

    def detect(self, roi_image, confidence):
        """Return configured disease boxes without a model-provided aim point."""
        result = self._model(roi_image, verbose=False, conf=float(confidence))[0]
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

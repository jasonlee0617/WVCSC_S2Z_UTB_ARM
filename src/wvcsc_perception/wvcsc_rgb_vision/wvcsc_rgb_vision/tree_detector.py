"""Tree-detection model adapter for the WVCSC perception pipeline."""

from pathlib import Path
import sys

from .model_utils import canonical_class_name, resolve_yolo_model_path, validate_yolo_model
from .perception_types import Instance


class TreeDetector:
    """Own one YOLO detect model and translate its boxes into ``Instance`` values."""

    def __init__(self, model_path, class_names, *, strict_model_classes=False):
        self._class_names = dict(class_names)
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
            model, 'detect', self._class_names, exact_names=strict_model_classes)
        return model

    def detect(self, image, confidence):
        result = self._model(image, verbose=False, conf=float(confidence))[0]
        if result.boxes is None:
            return []
        instances = []
        for box in result.boxes:
            class_name = canonical_class_name(int(box.cls[0]), result.names)
            if class_name not in self._class_names.values():
                continue
            left, top, right, bottom = [float(value) for value in box.xyxy[0].tolist()]
            instances.append(Instance(
                '', class_name, float(box.conf[0]), left, top, right, bottom,
                (left + right) / 2.0, (top + bottom) / 2.0))
        return instances

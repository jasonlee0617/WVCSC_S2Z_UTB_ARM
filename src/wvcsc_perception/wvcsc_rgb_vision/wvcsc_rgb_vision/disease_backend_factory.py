"""Construct the configured disease-target inference backend."""

from .disease_detector import DiseaseDetector
from .disease_segmenter import DiseaseSegmenter


def create_disease_backend(
        backend, model_path, target_class_id, target_class_name,
        *, strict_model_classes=False):
    """Return one validated ``segment`` or ``detect`` backend."""
    backend = str(backend).strip().lower()
    backend_types = {
        'segment': DiseaseSegmenter,
        'detect': DiseaseDetector,
    }
    try:
        backend_type = backend_types[backend]
    except KeyError as error:
        raise ValueError(
            'disease_model_backend must be "segment" or "detect"') from error
    return backend_type(
        model_path,
        target_class_id,
        target_class_name,
        strict_model_classes=strict_model_classes,
    )

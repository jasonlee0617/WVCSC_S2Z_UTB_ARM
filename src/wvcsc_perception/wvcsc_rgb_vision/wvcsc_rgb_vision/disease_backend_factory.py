# 中文说明：根据配置创建 segment 或 detect 病态目标后端。
# 工厂只负责选择/构造模型适配器，不参与树导航、目标账本或 ROS 消息发布。
"""Construct the configured disease-target inference backend."""

from .disease_detector import DiseaseDetector
from .disease_segmenter import DiseaseSegmenter


def create_disease_backend(
        backend, model_path, target_class_id, model_target_class_name,
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
        model_target_class_name,
        strict_model_classes=strict_model_classes,
    )

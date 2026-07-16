"""Model-path and independent class-map helpers for two-stage YOLO."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


TREE_CLASS_NAMES = {0: 'tree'}
FRUIT_CLASS_NAMES = {0: 'healthy_fruit', 1: 'diseased_fruit'}


def canonical_class_name(class_id, model_names):
    class_id = int(class_id)
    if isinstance(model_names, dict):
        return str(model_names.get(class_id, model_names.get(str(class_id), f'cls{class_id}')))
    if isinstance(model_names, (list, tuple)) and 0 <= class_id < len(model_names):
        return str(model_names[class_id])
    return f'cls{class_id}'


def validate_yolo_model(model, expected_task, expected_names):
    """Fail fast when a deployment weight has the wrong task or class map."""
    actual_names = ({int(key): str(value) for key, value in model.names.items()}
                    if isinstance(model.names, dict)
                    else {index: str(value) for index, value in enumerate(model.names)})
    if model.task != expected_task or actual_names != expected_names:
        raise ValueError(
            f'YOLO model contract mismatch: expected task={expected_task}, '
            f'names={expected_names}; found task={model.task}, names={model.names}')


def resolve_yolo_model_path(path_value):
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return str(path)
    return str(
        Path(get_package_share_directory('wvcsc_rgb_vision')) / 'models' / path)

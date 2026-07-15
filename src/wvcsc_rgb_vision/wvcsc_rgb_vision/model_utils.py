"""Small model helpers retained for the future YOLO-Seg backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


CLASS_NAMES = {0: 'disease_spot', 1: 'pest_cluster'}


def canonical_class_name(class_id, model_names=None):
    class_id = int(class_id)
    if class_id in CLASS_NAMES:
        return CLASS_NAMES[class_id]
    if isinstance(model_names, dict):
        return str(model_names.get(class_id, f'cls{class_id}'))
    if isinstance(model_names, (list, tuple)) and 0 <= class_id < len(model_names):
        return str(model_names[class_id])
    return f'cls{class_id}'


def resolve_yolo_model_path(path_value):
    path = Path(path_value).expanduser()
    if path.is_absolute():
        return str(path)
    return str(
        Path(get_package_share_directory('wvcsc_rgb_vision')) / 'models' / path)

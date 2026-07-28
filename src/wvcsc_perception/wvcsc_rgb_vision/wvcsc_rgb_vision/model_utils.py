# model_utils.py
"""
病态目标 YOLO 感知节点的模型路径解析与合规性检验工具。

职责：
1. 定义感知任务中使用的标准病态目标类别映射。
2. 在加载 YOLO 模型前进行“快速失败 (Fail-fast)”校验：确保模型的任务类型
    (detect/segment) 和类别名称完全符合预期。
3. 统一处理模型权重文件的路径（支持绝对路径与 ROS 包共享目录的相对路径）。
"""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory


# 病态目标在 ROS 输出中的固定类别名。模型原生类别名由
# ``model_target_class_name`` 单独配置，不参与下游消息契约。
CANONICAL_DISEASE_TARGET_CLASS_NAME = 'diseased_target'

# 权重文件保留各自训练时的原生类别名；ROS 输出统一为 diseased_target。
DISEASED_TARGET_CLASS_ALIASES = {
    'diseased_fruit': 'diseased_target',
    'disease_leaf': 'diseased_target',
    'diseased_target': 'diseased_target',
}
def validate_yolo_model(
        model, expected_task, expected_native_names, *, exact_names=False):
    """
    在节点初始化时强制校验 YOLO 模型的合规性（快速失败机制）。

    这是一道非常关键的“安全防线”。它防止因为训练脚本配置错误，导致部署的权重
    实际是一个“检测模型”而非“分割模型”，或者“类别名称”被意外改变（如
    病果的 ID 变成了 ID 2）。如果不加此校验，错误的模型可能在仿真运行几十秒后
    引发不可预测的下标越界或 `KeyError`。

    Args:
        model (ultralytics.YOLO): 已加载的 YOLO 模型对象。
        expected_task (str): 期望的任务类型 (如 'detect' 或 'segment')。
        expected_native_names (dict): 权重中目标类别 ID 到原生类别名称的映射。
        exact_names (bool): 为 True 时要求模型类别表完全相等。实机
            配置使用此严格模式，防止带有额外类别的权重被误部署。

    Raises:
        ValueError: 当模型的任务类型或类别名称与期望值不匹配时抛出异常。
    """
    # 统一模型的 names 字典格式（处理各种可能的数据结构）。
    actual_names = ({int(key): str(value) for key, value in model.names.items()}
                    if isinstance(model.names, dict)
                    else {index: str(value) for index, value in enumerate(model.names)})

    names_match = (
        actual_names == expected_native_names if exact_names
        else expected_native_names.items() <= actual_names.items())
    if model.task != expected_task or not names_match:
        raise ValueError(
            f'YOLO model contract mismatch: expected task={expected_task}, '
            f'native_names={expected_native_names}; found task={model.task}, '
            f'native_names={actual_names}')


def load_yolo_model(
        model_path, expected_task, expected_native_names, *, exact_names=False):
    """Load and validate one disease-model checkpoint once for every backend."""
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
        model, expected_task, expected_native_names, exact_names=exact_names)
    return model


def resolve_yolo_model_path(path_value):
    """
    解析 YOLO 权重文件的绝对路径。

    允许在 `vision_sim.yaml` 中通过相对路径（如 `yolov8s_seg_sim.pt`）
    引用位于该功能包 `share` 目录下的模型权重。

    Args:
        path_value (str): YAML 配置中读出的路径字符串。

    Returns:
        str: 可以在 YOLO 对象中直接加载的绝对路径字符串。
    """
    path = Path(path_value).expanduser()
    # 如果路径本身已经是绝对路径（如 `/absolute/path/weights.pt`），则直接返回
    if path.is_absolute():
        return str(path)
    # 如果是相对路径，将其解析为 ROS 包共享目录下的 `models` 子目录
    # 解析结果例如：/opt/ros/humble/share/wvcsc_rgb_vision/models/yolov8s_seg_sim.pt
    return str(
        Path(get_package_share_directory('wvcsc_rgb_vision')) / 'models' / path)

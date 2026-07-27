# WVCSC_S2Z_UTB_ARM 工控机 YOLO 依赖安装与 GPU 检查指南

本文档用于在小车工控机上部署真实 YOLO 推理环境。目标是让以下启动参数可用：

```bash
yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

本项目采用隔离 Python 虚拟环境运行 YOLO/PyTorch，同时通过 `--system-site-packages` 复用 ROS Humble 的 `rclpy`、`cv_bridge` 等系统包。

官方参考：

- PyTorch 安装选择器：https://docs.pytorch.org/get-started/locally/
- Ultralytics 安装文档：https://docs.ultralytics.com/quickstart/

注意：PyTorch 的 CUDA wheel 会随时间更新。本文给出检查方法和推荐流程；最终 `torch` 安装命令应以 PyTorch 官方安装选择器为准。

## 1. 基础系统信息检查

在工控机上执行：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash

lsb_release -a
uname -a
python3 --version
which python3
lscpu
free -h
df -h
```

记录这些信息：

- Ubuntu 版本；
- CPU 型号；
- 内存容量；
- 系统盘剩余空间；
- Python 版本。

ROS Humble 默认常见环境为 Ubuntu 22.04 + Python 3.10。若 Python 版本不是 3.10，先不要继续安装，先确认 ROS 环境是否正确。

## 2. 判断是否有 NVIDIA GPU

先查 PCI 设备：

```bash
lspci | grep -Ei "nvidia|vga|3d|display"
```

再查 NVIDIA 驱动：

```bash
nvidia-smi
```

可能结果：

- `nvidia-smi` 正常显示 GPU、Driver Version、CUDA Version：可以走 GPU 路线；
- `nvidia-smi: command not found`：可能没有 NVIDIA 驱动，继续查硬件；
- `No devices were found`：驱动存在但没有识别到 GPU；
- 没有 NVIDIA 设备：走 CPU 路线。

查 CUDA 工具链：

```bash
nvcc --version
dpkg -l | grep -Ei "nvidia|cuda|cudnn"
```

注意：PyTorch pip wheel 自带 CUDA 运行时，不要求系统安装完整 CUDA Toolkit；但必须有可用 NVIDIA 驱动。`nvidia-smi` 比 `nvcc` 更关键。

## 3. 判断安装路线

### 3.1 有 NVIDIA GPU

推荐路线：

1. 用 `nvidia-smi` 记录 Driver Version 和 CUDA Version；
2. 打开 PyTorch 官方安装选择器；
3. 选择 Linux、Pip、Python、CUDA；
4. 使用官方给出的 `pip install torch torchvision ...` 命令；
5. 再安装项目固定的 `numpy`、`opencv-python`、`ultralytics`。

不要盲目照搬另一台电脑的 CUDA wheel。比如本机曾验证过 `cu130`，但工控机驱动不一定支持。

### 3.2 没有 NVIDIA GPU

可以走 CPU 路线，先跑通功能和接口：

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

CPU 推理会慢很多，可能无法满足闭环实时性，但适合验证：

- ROS 包导入；
- YOLO 模型契约；
- C10 图像接入；
- 话题发布；
- launch 参数链。

## 4. 创建 YOLO 虚拟环境

不要用系统 Python 直接安装 YOLO。

```bash
mkdir -p "$HOME/venvs"
python3 -m venv --system-site-packages "$HOME/venvs/wvcsc_yolo_ros"
source "$HOME/venvs/wvcsc_yolo_ros/bin/activate"

python -m pip install --upgrade pip setuptools wheel
```

为什么必须加 `--system-site-packages`：

- `rclpy` 来自 ROS Humble 系统环境；
- `cv_bridge` 来自 ROS Humble 系统环境；
- YOLO 节点需要在同一个解释器中同时导入 ROS 和 Ultralytics。

## 5. 安装 PyTorch

### 5.1 GPU 路线

进入虚拟环境：

```bash
source "$HOME/venvs/wvcsc_yolo_ros/bin/activate"
```

根据 PyTorch 官方选择器执行对应命令。示例形式如下，实际命令以官网为准：

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/<CUDA_WHEEL>
```

`<CUDA_WHEEL>` 由 PyTorch 官方选择器给出，例如 `cu121`、`cu124`、`cu128` 或其他当前支持版本。

安装后验证：

```bash
python - <<'PY'
import torch
import torchvision
print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("cuda available:", torch.cuda.is_available())
print("cuda runtime:", torch.version.cuda)
print("gpu:", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")
PY
```

验收：

```text
cuda available: True
```

如果是 `False`，不要继续调 ROS。先处理 NVIDIA 驱动或 PyTorch wheel 匹配问题。

### 5.2 CPU 路线

```bash
source "$HOME/venvs/wvcsc_yolo_ros/bin/activate"
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

验证：

```bash
python - <<'PY'
import torch
import torchvision
print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("cuda available:", torch.cuda.is_available())
PY
```

CPU 路线预期：

```text
cuda available: False
```

## 6. 安装 Ultralytics 和项目运行依赖

进入虚拟环境：

```bash
source "$HOME/venvs/wvcsc_yolo_ros/bin/activate"
```

安装项目当前固定版本：

```bash
"$HOME/venvs/wvcsc_yolo_ros/bin/python" -m pip install \
  numpy==1.26.4 \
  opencv-python==4.11.0.86 \
  ultralytics==8.3.217 \
  "typing_extensions>=4.12,<5"
```

也可以参考 `wvcsc_perception/wvcsc_rgb_vision/requirements-yolo-runtime.txt`，但如果里面的 CUDA wheel 与工控机驱动不匹配，不要强行安装该文件。先按第 5 节安装匹配的 `torch/torchvision`，再安装上面的运行依赖。

验证 Ultralytics：

```bash
export PYTHONNOUSERSITE=1
export YOLO_CONFIG_DIR=/tmp/wvcsc_ultralytics

"$HOME/venvs/wvcsc_yolo_ros/bin/python" - <<'PY'
import cv2
import numpy
import torch
import torchvision
import ultralytics
print("cv2:", cv2.__version__)
print("numpy:", numpy.__version__)
print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("ultralytics:", ultralytics.__version__)
print("cuda available:", torch.cuda.is_available())
print("typing_extensions:", __import__("typing_extensions").__file__)
print("has deprecated:", hasattr(__import__("typing_extensions"), "deprecated"))
PY
```

注意：必须使用目标虚拟环境的 `python -m pip`。不要使用系统 `pip`，否则
`typing_extensions` 可能仍从 `/usr/lib/python3/dist-packages` 加载旧版本。

## 7. 验证 ROS 包能在 venv 中导入

必须先 source ROS 和工作区：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/WVCSC_S2Z_UTB_ARM/install/setup.bash"
export PYTHONNOUSERSITE=1
export YOLO_CONFIG_DIR=/tmp/wvcsc_ultralytics

"$HOME/venvs/wvcsc_yolo_ros/bin/python" - <<'PY'
import rclpy
import cv_bridge
import sensor_msgs.msg
import vision_msgs.msg
import wvcsc_interfaces.msg
import torch
import ultralytics
import typing_extensions
print("ROS + YOLO runtime ok")
print("torch:", torch.__version__)
print("cuda:", torch.cuda.is_available())
print("typing_extensions:", typing_extensions.__file__)
print("has deprecated:", hasattr(typing_extensions, "deprecated"))
PY
```

如果这里报 `No module named rclpy` 或 `No module named cv_bridge`，通常是虚拟环境创建时没有使用 `--system-site-packages`。删除 venv 后重建：

```bash
rm -rf "$HOME/venvs/wvcsc_yolo_ros"
python3 -m venv --system-site-packages "$HOME/venvs/wvcsc_yolo_ros"
```

## 8. 部署真实权重

将训练好的权重复制到：

```text
~/WVCSC_S2Z_UTB_ARM/src/wvcsc_perception/wvcsc_rgb_vision/models/yolov8s_seg_real.pt
```

重新构建安装：

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select wvcsc_rgb_vision
source install/setup.bash
```

确认安装后的权重存在：

```bash
ls -l "$(ros2 pkg prefix wvcsc_rgb_vision)/share/wvcsc_rgb_vision/models/yolov8s_seg_real.pt"
```

## 9. 校验模型契约

执行：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/WVCSC_S2Z_UTB_ARM/install/setup.bash"
export PYTHONNOUSERSITE=1
export YOLO_CONFIG_DIR=/tmp/wvcsc_ultralytics

"$HOME/venvs/wvcsc_yolo_ros/bin/python" - <<'PY'
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

model_dir = Path(get_package_share_directory("wvcsc_rgb_vision")) / "models"
contracts = [("yolov8s_seg_real.pt", "segment", {0: "disease_leaf"})]

for filename, expected_task, expected_names in contracts:
    model = YOLO(str(model_dir / filename))
    names = model.names
    actual_names = (
        {int(k): str(v) for k, v in names.items()}
        if isinstance(names, dict)
        else {i: str(v) for i, v in enumerate(names)}
    )
    print(filename, "task=", model.task, "names=", actual_names)
    if model.task != expected_task or actual_names != expected_names:
        raise SystemExit(
            f"{filename}: expected task={expected_task}, names={expected_names}; "
            f"found task={model.task}, names={actual_names}"
        )

print("model contracts ok")
PY
```

通过标准：

```text
model contracts ok
```

## 10. 推理冒烟测试

有 C10 图像时，先启动机械臂单独测试栈：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

另开终端检查：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic list | grep /vision
```

在单臂 Qt 填写树 X/Y/Z 与喷洒时长，点击“执行单目标喷洒”。Action 阶段、进度、结果以及
`/camera/color/image_raw`、YOLO 调试图像都在同一窗口显示。

实机喷洒前必须同时确认继电器服务已启动。`real_arm_spray_test.launch.py` 会自动
启动 `controller_pkg`，实机执行器使用 `service` 模式调用第 2 路；机械臂串口
`serial_port` 与继电器配置文件中的 `PortName` 是两条独立串口：

```bash
ros2 service type /relay/set
# wvcsc_interfaces/srv/SetRelay
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: true, duration: 1.0}"
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: false, duration: 0.0}"
```

确认第 2 路实际吸合和断开后，再通过单臂 Qt 执行喷洒。如果继电器串口不是默认的
`/dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.0-port0`，复制并修改
`controller_pkg/config/fault.ini`，然后通过
`relay_config_file:=/绝对路径/relay_fault.ini` 传给 launch。

检查：

```bash
ros2 topic hz /vision/diseased_target_debug_image
ros2 topic echo /vision/target
```

## 11. 常见问题

### 11.1 `torch.cuda.is_available()` 为 False

按顺序检查：

```bash
nvidia-smi
python - <<'PY'
import torch
print(torch.__version__)
print(torch.version.cuda)
print(torch.cuda.is_available())
PY
```

判断：

- `nvidia-smi` 失败：先装或修 NVIDIA 驱动；
- `nvidia-smi` 正常但 PyTorch CUDA 为 False：PyTorch 装成了 CPU 版，按官方选择器重装 CUDA wheel；
- 驱动 CUDA Version 太低：升级驱动或选择兼容的 PyTorch CUDA wheel。

### 11.2 `No module named cv_bridge`

原因：venv 没有继承 ROS 系统包。

处理：

```bash
deactivate 2>/dev/null || true
rm -rf "$HOME/venvs/wvcsc_yolo_ros"
python3 -m venv --system-site-packages "$HOME/venvs/wvcsc_yolo_ros"
```

然后重新安装 PyTorch、Ultralytics 和运行依赖。

### 11.3 `YOLO_CONFIG_DIR` 权限问题

固定使用：

```bash
export YOLO_CONFIG_DIR=/tmp/wvcsc_ultralytics
mkdir -p "$YOLO_CONFIG_DIR"
```

### 11.4 `torchvision` 与 `torch` 版本不匹配

### 11.5 `ImportError: cannot import name 'deprecated' from typing_extensions`

这表示 YOLO 使用的 Python 虚拟环境加载了系统旧版 `typing_extensions`。
这不是 C10 相机、ROS 话题或 YOLO 权重问题。不要修改相机参数，也不要直接
使用系统 `pip`；使用与 launch 完全相同的解释器修复：

```bash
VENV="$HOME/venvs/wvcsc_yolo_ros"

"$VENV/bin/python" -m pip install \
  --upgrade \
  --force-reinstall \
  "typing_extensions>=4.12,<5"

PYTHONNOUSERSITE=1 "$VENV/bin/python" - <<'PY'
import typing_extensions
print("typing_extensions:", typing_extensions.__file__)
print("has deprecated:", hasattr(typing_extensions, "deprecated"))
PY
```

验收时 `typing_extensions.__file__` 应位于：

```text
~/venvs/wvcsc_yolo_ros/lib/python3.10/site-packages/
```

不能继续指向：

```text
/usr/lib/python3/dist-packages/typing_extensions.py
```

`--system-site-packages` 仍需保留，因为 `rclpy` 和 `cv_bridge` 来自 ROS
系统环境；只需在 venv 中覆盖这个不兼容的 Python 包。

现象通常是导入 `torchvision` 时报错。处理方式：

```bash
pip uninstall -y torch torchvision torchaudio
```

然后按 PyTorch 官方选择器重新安装同一 CUDA wheel 下的一组 `torch` 和 `torchvision`。

### 11.5 工控机没有 GPU

可以安装 CPU 版先验证接口，但要降低预期：

- 图像推理可能明显低于实时；
- VisualServo 目标更新可能变慢；
- 可用于验证权重契约、话题、launch、C10 图像链；
- 最终闭环性能仍建议使用 NVIDIA GPU。

## 12. 推荐记录模板

现场完成后，把以下输出保存到测试记录：

```bash
lsb_release -a
uname -a
python3 --version
lscpu | grep -E "Model name|CPU\\(s\\)"
free -h
df -h "$HOME"
lspci | grep -Ei "nvidia|vga|3d|display"
nvidia-smi
source "$HOME/venvs/wvcsc_yolo_ros/bin/activate"
python - <<'PY'
import torch, torchvision, ultralytics, cv2, numpy
print("torch", torch.__version__, "cuda", torch.version.cuda, torch.cuda.is_available())
print("torchvision", torchvision.__version__)
print("ultralytics", ultralytics.__version__)
print("opencv", cv2.__version__)
print("numpy", numpy.__version__)
PY
```

如果后续需要复现问题，这些信息比“有 GPU/没 GPU”的口头描述更可靠。

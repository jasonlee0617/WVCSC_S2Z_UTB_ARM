# WVCSC 视觉感知解耦与病态模型接入指南

本文档说明 WVCSC 的病态目标模型如何接入、验证和回退。它只描述感知架构与模型行为；Python、CUDA、PyTorch、Ultralytics 的安装仍以 [工控机 YOLO 依赖安装与 GPU 检查指南](WVCSC_S2Z_UTB_ARM_工控机YOLO依赖安装与GPU检查指南.md) 为准。

## 1. 架构与不变边界

```text
/camera/.../image_raw
        │
        ▼
TreeDetector（固定：tree detect 权重、类别、ROI 选择与锁树逻辑）
        │ tree bounding box
        ▼
PerceptionPipeline（树 ROI 扩边、裁剪）
        │ ROI-local BGR image
        ├───────────── DiseaseSegmenter（segment） ── mask-safe point ─┐
        └───────────── DiseaseDetector（detect） ──── box centre ─────┤
                                                                        ▼
PerceptionPipeline（ROI 坐标回填、去重、ID 跟踪、模板回退、排序/最多两目标）
        │
        ├── /vision/diseased_target_detections
        ├── /vision/diseased_target_debug_image
        ├── /vision/perception_debug
        └── /vision/target (Target2D)
                                      │
                                      ▼
                         视觉伺服 → 单臂喷洒 → HOME
```

树检测不属于本次可替换范围：`TreeDetector`、树模型权重、树类别、树 ROI 选择和锁树逻辑均不应为了测试病态模型而修改。病态模型默认仍是分割模型；只有将 `disease_model_backend` 显式改为 `detect` 时，才会运行检测模型。

## 2. 三个模块的职责和数据契约

| 模块 | 负责内容 | 不负责内容 |
| --- | --- | --- |
| `tree_detector.py` | 全图树检测，输出树框 | 病态模型、ROI、病叶 ID、喷洒点 |
| `disease_segmenter.py` / `disease_detector.py` | 对输入的树 ROI 推理，输出病态框；分割额外输出 ROI 内掩膜安全点 | 裁剪原始图、全图坐标、ROS、ID 跟踪、发布 |
| `perception_pipeline.py` | ROI 扩边/裁剪、坐标回填、去重、ID、模板回退、最多目标限制、调试图和 ROS 话题 | 修改树模型或猜测检测模型的掩膜 |

病态后端的最小协议在 `wvcsc_perception/wvcsc_rgb_vision/wvcsc_rgb_vision/disease_target_backend.py` 中定义：输入是树 ROI 图像和置信度阈值；输出每个目标的 `class_name`、`confidence`、`left/top/right/bottom`，以及可选的 `control_u/control_v`。所有坐标在该阶段都是 **ROI 局部坐标**。

`PerceptionPipeline` 用树框和 `roi_padding` 计算 `(x0, y0, x1, y1)`，将后端的每个坐标回填为全图坐标：

```text
full_u = roi_u + x0
full_v = roi_v + y0
```

因此，后端实现绝不能自行加树 ROI 偏移；否则会产生二次偏移。后端也不能发布 ROS 消息。

## 3. segment 与 detect 的控制点语义

`Target2D` 消息接口不变，视觉伺服继续只读取 `center_u`、`center_v`、`width`、`height`。

| 后端 | 权重任务 | `Target2D.center_u/v` | 适用说明 |
| --- | --- | --- | --- |
| `segment`（默认） | `segment` | 掩膜内部距离边界较远的安全点 | 保持当前实机/仿真的喷洒瞄准行为 |
| `detect`（实验） | `detect` | 检测框中心：`((left+right)/2, (top+bottom)/2)` | 没有掩膜，不能伪造安全点 |

检测模型的后端不计算、不输出 `aim_u/v`。框中心由流水线在全图坐标回填时计算。调试 JSON 和调试图仍保留 `aim_uv` 字段/标记；在 detect 模式它表示框中心，在 segment 模式它表示掩膜安全点。

## 4. 当前默认行为

配置文件：

- 实机：`wvcsc_perception/wvcsc_rgb_vision/config/vision_real.yaml`
- 仿真：`wvcsc_perception/wvcsc_rgb_vision/config/vision_sim.yaml`

两者的默认值均为：

```yaml
disease_model_backend: segment
fruit_model_path: yolov8s_seg_real.pt  # 仿真为 yolov8s_seg_sim.pt
target_class_id: 0
target_class_name: diseased_target
strict_model_classes: true
```

实机仍按置信度只发布最高两个病态目标（`max_diseased_targets: 2`）；仿真保持不限制（`0`）。ROI 扩边、`fruit_confidence`、固定分割 IoU `0.45`、目标排序、跟踪 ID、模板回退和所有 `/vision/*` 话题均保持原有语义。

## 5. 接入新的病态模型

### 5.1 准备权重

权重可以放入 `wvcsc_perception/wvcsc_rgb_vision/models/`，并填写相对文件名；也可以填写绝对路径。相对路径会解析为该 ROS 包的 `models/` 目录。

接入前先确认 Ultralytics 导出的 `model.task`：分割必须是 `segment`，检测必须是 `detect`。检测权重不能用分割权重替代，反之亦然。

### 5.2 保持默认分割（推荐生产配置）

```yaml
# vision_real.yaml 或 vision_sim.yaml
disease_model_backend: segment
fruit_model_path: yolov8s_seg_real.pt
target_class_id: 0
target_class_name: diseased_target
strict_model_classes: true
fruit_confidence: 0.25
```

此配置保持掩膜安全点输出。不要把 segment 模型换成 `detect` 权重后仅修改路径；启动时的模型任务校验应当失败，这是预期的保护行为。

### 5.3 接入 detect 实验模型

单类别 detect 权重示例：

```yaml
disease_model_backend: detect
fruit_model_path: /absolute/path/disease_leaf_detect.pt
target_class_id: 0
target_class_name: diseased_target
strict_model_classes: true
fruit_confidence: 0.25
```

多类别 detect 权重示例（目标病叶是权重中的 class 2）：

```yaml
disease_model_backend: detect
fruit_model_path: /absolute/path/multiclass_leaf_detect.pt
target_class_id: 2
target_class_name: diseased_target
strict_model_classes: false
fruit_confidence: 0.25
```

多类别模型必须把 `strict_model_classes` 设为 `false`，否则额外类别会被拒绝。即使关闭严格类别表校验，节点仍会验证任务必须为 `detect`，并验证 `target_class_id` 存在且对应已知病态目标标签（`disease_leaf`、`diseased_fruit` 或 `diseased_target`）。如果新训练集用了其他类别名，应先在 `model_utils.py` 的病态标签别名表中显式加入该别名，并补充测试；不要通过关闭校验把未知类别静默当成病叶。

### 5.4 如需接入非 YOLO 后端

实现 `DiseaseTargetBackend.detect(roi_image, confidence)`，返回 ROI 局部的 `DiseaseTarget` 列表。分割类算法可提供 `control_u/control_v`；纯检测算法必须留空，流水线会使用框中心。模型适配器不应修改树逻辑，也不应访问 ROS。接入后在 `PerceptionPipeline` 的后端选择处显式注册一种新的配置值，并为其补充任务/类别校验和下列验收测试。

## 6. 构建、启动与回退

```bash
cd ~/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --packages-select wvcsc_rgb_vision wvcsc_bringup wvcsc_simulation --symlink-install
source install/setup.bash
```

仿真 YOLO 流程：

```bash
ros2 launch wvcsc_simulation system_sim.launch.py use_mock_targets:=false
```

实机单臂测试（使用隔离 YOLO 解释器）：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="$HOME/venvs/wvcsc_yolo_ros/bin/python"
```

先在仿真或相机观察模式完成检测比较，再允许喷洒。要回退到生产分割，只恢复：

```yaml
disease_model_backend: segment
fruit_model_path: yolov8s_seg_real.pt  # 仿真改为 yolov8s_seg_sim.pt
target_class_id: 0
strict_model_classes: true
```

## 7. 验收清单

- 默认 segment：同一输入图像下，ROI、权重、`fruit_confidence`、IoU、病态框、掩膜安全点、目标 ID、排序、调试图和 `/vision/target` 与改造前一致。
- detect：全图框坐标等于 ROI 局部框加 `(x0, y0)`；`Target2D.center_u/v` 严格等于该全图框中心。
- 实机：仍最多发布两个最高置信度目标；仿真仍允许一个或多个目标。
- 任务：多目标队列、视觉伺服失败后的回退喷洒、HOME、下一目标出队流程均使用同一 `Target2D` 接口。
- 仅当 C10 实机图像和完整单臂流程均通过后，detect 模型才可作为喷洒模型；离线框指标不能替代这一验证。

## 8. 常见错误

| 现象 | 原因与处理 |
| --- | --- |
| 启动报模型任务不匹配 | `disease_model_backend` 与权重任务不一致；检查 `segment`/`detect`。|
| 启动报类别契约不匹配 | `target_class_id` 不存在、不是病态标签，或多类别权重错误地开启了严格模式。|
| 启动报权重不存在 | 相对权重必须在 ROS 包 `models/`，否则改用绝对路径。|
| 框位置整体偏移 | 后端输出了全图坐标或重复加了 ROI 偏移；后端只能返回 ROI 局部坐标。|
| detect 的瞄准表现与 segment 不同 | 这是必然语义差异：detect 使用框中心，不具备掩膜安全点。不要把框中心误称为掩膜安全点。|
| 调试图有框但没有喷洒 | 除视觉目标外，还须检查任务阶段、选中 ID、视觉伺服与机械臂安全门控。|

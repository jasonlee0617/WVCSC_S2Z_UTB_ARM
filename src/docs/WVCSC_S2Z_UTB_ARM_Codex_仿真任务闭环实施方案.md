# WVCSC ARMSpray 视觉喷洒闭环实施方案

> 更新日期：2026-07-18
> 当前阶段：两级 YOLO、动态观察/重心、20 Hz MoveIt Servo 和病果逐个喷洒链路均已接通；最新 `wvcsc_servo_20260718_1421` 尚未完成喷洒，本轮已实施增益补偿、连续目标失效判定和重心后可靠性门控，等待 Gazebo 复验。
> 验证结果：本轮 `wvcsc_visual_servo`、`wvcsc_arm_task` 隔离构建通过，93 项相关测试通过。动态观察位姿、目标重心、检测去重、对准恢复和人工安全复位均已实现。

## 1. 目标与边界

### 1.1 本次目标

完成一棵树的完整视觉喷洒闭环：

```text
MOVING_TO_OBSERVE → SCANNING_TREE → DETECTING_FRUITS → QUEUING
    → ALIGNING → SPRAYING → RETURNING_TO_OBSERVE → 复检直到队列为空 → HOME
```

Mock 模式已于 2026-07-17 移除，`perception_mode` 参数和 `mock_vision` 节点已删除。验收目标：四棵树全部按上述七阶段链路完成，每棵树最少喷洒 1 个病果，连续运行三轮无安全锁定。

### 1.2 明确不纳入本次的范围

- 真实无人机飞控与视觉识别
- 真实 C10 相机内参/手眼标定
- 真实喷洒泵阀硬件
- Web UI
- 多无人机调度

---

## 2. ARM_SPRAYING 七阶段详解

### 2.1 整体架构

```
mission_manager (一棵树)
    │
    ├─ tree_hint: PoseStamped (果树在 alicia_base_link 下的坐标)
    └─ ExecuteSpray.Goal → spray_task
                              │
                              ├─ MoveIt2 (plan + execute)
                              ├─ YOLO 两级推理 (tree detect + fruit seg)
                              ├─ moveit_servo (IBVS XY 对准)
                              └─ spray_controller (Spray Action)
```

### 2.2 [1] MOVING_TO_OBSERVE — 动态观察位姿

**不再使用固定关节角度**。每个观察位姿都根据果树的实际位置动态计算。

**计算链**：

```
mission_manager 发送 tree_hint (PoseStamped, 在 alicia_base_link 下)
    ↓
spray_task._move_to_observation():
    1. TF 查询 tree_hint 在 base_frame 下的坐标 → tree_in_base = (tx, ty, tz)
    2. TF 查询 tool0 → camera_color_optical_frame 的外参 → camera_mount
    3. 从 observation_distance_candidates 依次尝试距离
    4. camera_look_at_pose(tree_in_base, aim_height, camera_height, distance)
       → 相机光轴(+Z)指向树冠位置，得到 camera 位姿
    5. tool_pose_from_camera_pose() → 转为 tool0 位姿
    6. yaw_rotate_quaternion() 生成 scan_poses (扇形扫描候选)
    7. arm.move_pose() → MoveIt IK + 规划 + 执行
    ↓
机械臂到达观察位姿，相机光轴对准树冠
```

**关键参数**（`arm_task.yaml`）：

```yaml
tree_aim_height: 1.20              # 相机瞄准树冠高度 (m, 相对树根)
camera_observation_height: 1.90    # 相机安装高度 (m, 相对树根)
observation_distance_candidates:    # 尝试距离列表，依次尝试直到找到一个 IK 可解
  [1.10, 1.00, 1.20, 0.90, 1.30, 1.40, 1.50]
scan_yaw_offsets_deg:               # 扇形扫描的相机偏航角偏移
  [0.0, -10.0, 10.0, -20.0, 20.0]
```

**为什么距离是候选列表？** 不同距离对应的 IK 解可达性不同。太近可能碰撞、太远可能超出机械臂工作空间。按列表顺序尝试，第一个规划成功的距离被采纳。如果对准失败，还可以自动尝试下一候选距离（`_recover_to_next_observation`）。

### 2.3 [2] SCANNING_TREE — 果树确认

```
到达观察位姿
    ↓
spray_task._scan_for_tree():
    for each scan_pose (从 yaw=0° 开始，依次尝试 ±10°, ±20°):
        1. arm.move_pose(scan_pose) → 移动相机朝向
        2. 发布 /vision/inference_mode = "tree"
        3. 等待 tree_detections 连续 confirmation_frames=3 帧
        4. 找到 → 记录当前 pose 为 observation_pose，返回成功
        5. 未找到 → 尝试下一个 scan_pose
    ↓
全部 scan_pose 尝试完毕仍未找到 → VISION_FAILED → return HOME
```

**扫描参数**：

```yaml
scan_pose_detection_timeout_sec: 1.0   # 每个扫描姿态最多等待 1 秒
confirmation_frames: 3                 # 需要连续 3 帧确认
tree_confidence: 0.50                  # tree 检测置信度阈值
```

`scan_yaw_offsets_deg` 定义了 5 个扫描姿态（0° 标称 + 左右各 2 个偏角），每个姿态最多等 1 秒，总计最多约 5 秒。

### 2.4 [3] DETECTING_FRUITS — 果实实例分割

```
tree 确认后
    ↓
发布 /vision/inference_mode = "fruits"
    ↓
YOLOv8n-seg 推理 (tree + healthy_fruit + diseased_fruit)
    ↓
spray_task._wait_for_fruits():
    1. 等待 fruit_detections 连续 confirmation_frames=3 帧
    2. 只接受连续出现 ≥confirmation_frames 帧的候选
       (过滤单帧误检)
    ↓
返回候选列表 (只含 diseased_fruit 类)
```

**参数**：

```yaml
detection_timeout_sec: 5.0                  # 最多等 5 秒
fruit_confidence: 0.30                      # 任务队列准入阈值
target_post_recenter_min_confidence: 0.30   # 重心后 Servo 准入阈值
target_post_recenter_stable_sec: 0.50       # 连续可靠时长
```

**两类 YOLO 模型的职责分工**：

| 模型 | 任务 | 输入 | 输出 |
|------|------|------|------|
| `wvcsc_tree_yolov8n.pt` | 目标检测 (Detect) | 完整 1280×720 图像 | `tree` bbox |
| `wvcsc_fruit_yolov8n_seg.pt` | 实例分割 (Seg) | 完整图像（tree ROI 由下游过滤） | `healthy_fruit` mask + `diseased_fruit` mask |

### 2.5 [4] QUEUING — 病果队列

```
候选列表 (diseased_fruit 的 Detection2D 消息)
    ↓
spray_task._queue():
    1. 过滤已处理 (processed) 和已耗尽 (exhausted) 的目标
       - IoU ≥ processed_iou_threshold (0.30) → 已处理
       - 中心距离 ≤ processed_center_distance_px (40px) → 已处理
    2. 帧内防御性去重
       - IoU ≥ 0.35 或中心距离 ≤ 10px → 同一物理果实
       - 只保留置信度最高的实例
    3. 按距离图像中心的远近排序 (近的优先)
    4. 同距离按置信度降序
    ↓
返回排序后的待喷洒队列
```

**队列不跨帧持久化**（第一版简化设计）：每轮 `DETECTING_FRUITS → QUEUING` 是独立的。去重依赖 `processed` 列表（已喷洒成功的）和 `exhausted` 列表（对准失败的）。

**去重参数**：

```yaml
processed_iou_threshold: 0.30
processed_center_distance_px: 40.0
```

### 2.6 [5] ALIGNING — IBVS 对准

```
队列首目标 (FruitTarget)
    ↓
spray_task._align_target():
    1. 发布 /vision/selected_target_id → target.target_id
    2. 发布 /vision/inference_mode = "target"
    3. 发送 AlignTarget.Goal 到 /vision/align_target Action
    4. 等待结果 (timeout = vision_timeout_sec = 8.0s)
    ↓
成功 → 进入 SPRAYING
失败 → 判断失败类型:
    ├─ TIMEOUT / TARGET_STALE / SERVO_SINGULARITY (可恢复)
    │   ├─ max_alignment_attempts=2 次内重试 → 尝试下一观察距离
    │   └─ 重试耗尽 → 标记 exhausted，继续下一目标
    ├─ SERVO_SAFETY_STOP → 触发 motion stop → INTERNAL_ERROR → 锁定
    └─ 其他 → VISION_FAILED → 尝试 HOME
```

**对准重试 + 观察距离恢复**：

```
对准失败 (可恢复类型)
    ↓
_recover_to_next_observation():
    1. _move_to_next_observation() → 尝试下一个 observation_distance_candidates
    2. _scan_for_tree() → 重新确认 tree
    3. 重置 fruit 跟踪 → 重新 DETECTING_FRUITS
    4. 用 pending_attempt 机制保留上次对准的 target_id
    ↓
继续对准（同一目标，新观察位姿）
```

**为什么需要观察距离恢复？** 某些距离下末端靠近奇异点或关节限位，导致 Servo 无法正常微调。换一个观察距离可能有更好的运动学条件。

### 2.7 [6] SPRAYING — 喷洒执行

```
对准成功
    ↓
发布 /vision/inference_mode = "idle"  (停止推理)
    ↓
spray_task._spray_target():
    1. 发送 Spray.Goal 到 /spray/execute Action
       (mission_id, tree_id, duration=spray_duration, mode="continuous")
    2. 等待结果
    ↓
成功 → 加入 processed 列表，sprayed 计数 +1
失败 → SPRAY_FAILED → 尝试 HOME
```

### 2.8 [7] RETURNING_TO_OBSERVE / HOME — 复检

```
喷洒完成
    ↓
return_to_observation() → 移回当前 observation_pose
    ↓
reset_fruit_tracking() → 清空帧计数
    ↓
回到 DETECTING_FRUITS → 重新检测
    ↓
    ├─ 仍有未处理 diseased_fruit → QUEUING → ALIGNING → ...
    └─ 队列为空 → RETURNING_HOME

HOME:
    arm.move_joints([0,0,0,0,0,0])
    ↓
COMPLETED / PARTIAL_SUCCESS / INSPECTED_NO_DISEASE
```

**为什么每次喷洒后要回到观察姿态？** 视觉伺服可能使机械臂偏离观察姿态（尤其是多目标连续对准时）。回到标称观察位姿保证下一次检测时相机朝向一致，避免累积漂移。

---

## 3. 状态机完整定义

### 3.1 ExecuteSpray Feedback Phase

```text
MOVING_TO_OBSERVE (0.05)
    → SCANNING_TREE (0.15)
    → DETECTING_FRUITS (0.25)
    → QUEUING (0.35)
    → ALIGNING (0.45) ← 逐目标循环
    → SPRAYING (0.60)
    → RETURNING_TO_OBSERVE (0.75)
    → 回到 DETECTING_FRUITS / RETURNING_HOME (0.90)
    → COMPLETED (1.00)
```

### 3.2 ExecuteSpray Result Codes

| Code | 含义 | 触发条件 |
|------|------|---------|
| OK (0) | 全部喷洒成功 | 至少 1 个病果喷洒成功，无失败/跳过 |
| PARTIAL_SUCCESS (10) | 部分成功 | 至少 1 个成功 + 至少 1 个跳过 |
| INSPECTED_NO_DISEASE (11) | 无病果 | tree 确认但未检测到 diseased_fruit |
| INVALID_GOAL (1) | 目标非法 | tree_hint 缺失或参数越界 |
| BUSY (2) | 正在执行 | 上一 Goal 未完成 |
| LOCKED (3) | 运动锁定 | motion_control 已发出 stop 锁 |
| OBSERVE_FAILED (4) | 观察位姿失败 | 所有距离候选 IK 均失败 |
| CANCELED (5) | 被取消 | 用户或 mission_manager 取消 |
| HOME_FAILED (6) | HOME 失败 | HOME 运动规划或执行失败 |
| INTERNAL_ERROR (7) | 内部错误 | Servo 安全停止等不可恢复错误 |
| VISION_FAILED (8) | 视觉失败 | tree 未找到或对准不可恢复 |
| SPRAY_FAILED (9) | 喷洒失败 | Spray Action 失败 |

---

## 4. 接口边界

### 4.1 输入

| 接口 | 类型 | 内容 |
|------|------|------|
| `/uav/disease_trees` | `DiseaseTreeArray` | 树级任务列表（map 坐标 + spray_side） |
| `tree_hint` (ExecuteSpray Goal) | `PoseStamped` | 果树在 `alicia_base_link` 下的位姿 |
| `/vision/tree_detections` | `Detection2DArray` | tree 检测候选 (class_id='tree') |
| `/vision/fruit_detections` | `Detection2DArray` | 果实分割候选 (class_id='diseased_fruit') |
| `/motion_control/locked` | `Bool` | 运动锁状态 |

### 4.2 输出

| 接口 | 类型 | 内容 |
|------|------|------|
| `/arm/execute_spray` | `ExecuteSpray` Action | 一棵树完整作业 |
| `/vision/align_target` | `AlignTarget` Action (调用) | 单病果 XY 对准 |
| `/spray/execute` | `Spray` Action (调用) | 喷洒执行 |
| `/vision/inference_mode` | `String` | 推理模式切换: idle / tree / fruits / target |
| `/vision/selected_target_id` | `String` | 当前对准的病果 ID |
| `/motion_control/command` | `String` | stop 指令（安全故障时） |

### 4.3 推理模式切换

```
idle     → 无活跃推理（导航中 / HOME / SPRAYING 时）
tree     → 仅 tree Detect（SCANNING_TREE 阶段）
fruits   → tree Detect + fruit Seg（DETECTING_FRUITS / QUEUING 阶段）
target   → 全推理 + 发布 selected_target_id（ALIGNING 阶段）
```

模式切换的作用：减少不必要推理（省 GPU/CPU），确保下游节点知道当前阶段以调整行为。

---

## 5. 构建与启动

### 5.1 构建

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to wvcsc_simulation
source install/setup.bash
```

### 5.2 仿真启动（YOLO 模式，权重已就绪）

```bash
./run_system_sim.sh
# 脚本内容：
#   source install/setup.bash
#   ros2 launch wvcsc_simulation system_sim.launch.py

# 或手动指定参数：
ros2 launch wvcsc_simulation system_sim.launch.py \
  yolo_python_executable:=/home/robot/venvs/wvcsc_yolo_ros/bin/python \
  auto_start_mission:=false
```

> `perception_mode` 参数已移除（2026-07-17），`mock_vision` 节点已删除。YOLOv8n 树检测权重已部署为唯一感知模式。

### 5.4 监控

```bash
# 任务状态
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /mission/status

# 任务计划（含作业位姿）
ros2 topic echo --once --qos-reliability reliable --qos-durability transient_local /mission/plan

# 活跃 Action
ros2 action list | grep -E 'navigate_to_pose|execute_spray|align_target'

# 里程计频率
ros2 topic hz /odom
```

---

## 6. 验收标准

### 6.1 仿真验收（YOLO 模式）

- [x] `yolo_python_executable` 默认指向 venv，启动不报错
- [x] SCANNING_TREE 阶段 YOLOv8n 检测到 tree（mAP50=0.995）
- [ ] DETECTING_FRUITS 阶段 YOLOv8n-seg 检测到 diseased_fruit（待果实分割权重训练）
- [ ] 病果队列按距离排序正确
- [ ] 每个病果的 IBVS 在 8s 内对准（双轴误差 ≤4px 且连续保持 ≥0.5s）
- [ ] Spray Action 调用成功，sprayed 计数正确
- [ ] 复检逻辑正确（喷洒后回到观察姿态重新检测）
- [ ] 无 Servo 安全锁定或碰撞
- [ ] `/mission/plan` 四个作业位姿正确
- [ ] Nav2 按输入顺序到达四棵树，每次到站后 `/odom` 停稳 1 秒
- [ ] 最终 `MISSION_COMPLETED`，`completed_targets=4`
- [ ] 连续三轮无异常

---

## 7. 下一步工作顺序

```
1. 完成 YOLO 权重训练
   - tree Detect: 数据集 wvcsc_tree_detect (24 train / 6 val)
   - fruit Seg: 数据集 wvcsc_fruit_seg (24 train / 6 val)
   - 验收: Mask precision/recall ≥ 0.80, mAP50 ≥ 0.70

2. YOLO 模式 Gazebo 单树闭环
   - perception_mode:=yolo auto_start_mission:=false
   - 手动触发一棵树，验证七阶段全部日志正确

3. YOLO 模式 Gazebo 四树连续闭环
   - 四树全自动，连续三轮

4. C10 实机到位后
   - 内参/手眼标定 → 实机 YOLO → 实机 Servo → 实机喷洒
```

---

## 8. 关键设计决策

### 8.1 观察位姿动态计算（替代固定关节角度）

**旧方案**：`observe_left_pose` / `observe_right_pose` 固定的 6 关节角度。
**新方案**：`tree_hint → camera_look_at_pose → tool_pose_from_camera_pose → MoveIt IK`。

优势：
- 不同树位置、不同株距自动适配
- 支持观察距离候选列表（IK 失败时自动尝试其他距离）
- 扇形扫描（yaw offset）由 `scan_yaw_offsets_deg` 参数化

### 8.2 task_spray 内部设计

- `tree_hint` 从 mission_manager 传入，已变换到 `alicia_base_link` 坐标系
- 观察位姿动态生成，不依赖固定关节角度
- `observation_distance_candidates` 优先列表中距离越近越好，依次回退
- 每个观察位姿附带一组 `scan_poses`（yaw 偏移），以应对果树位置和地图的误差

### 8.3 两级 YOLO 模型

| 级 | 任务 | 模型 | 类别 |
|----|------|------|------|
| 第一级 | 果树检测 | YOLOv8n Detect | `tree` (0) |
| 第二级 | 果实实例分割 | YOLOv8n-seg Seg | `healthy_fruit` (1), `diseased_fruit` (2) |

分离的理由：
- tree Detect：快速判断果树是否在视野中（不需要 mask）
- fruit Seg：精确的像素级分割用于 IBVS 对准（需要 mask 质心）

### 8.4 单目 RGB 边界

- IBVS 只修正图像平面 X/Y，不发 Z 轴速度
- 喷洒距离由停车位姿和观察距离参数保证（不虚构单目深度）
- 仅当 Servo 正常、目标可信、喷洒就绪时开阀

---

## 9. 数据集与模型

### 9.1 数据采集与训练状态

已于 2026-07-16 采集 30 张 C10 模拟图像（1280×720 PNG）：

- **树检测** (wvcsc_tree_detect)：24 train / 6 val，Labelme 手动标注完成。YOLOv8n 训练完成（epochs=20, lr0=0.001），验证集 mAP50=0.995, mAP50-95=0.941。权重已部署至 `wvcsc_rgb_vision/models/wvcsc_tree_yolov8n.pt`。
- **果实分割** (wvcsc_fruit_seg)：24 train / 6 val，图像已采集，`labels/` 目录为空，待人工标注。训练脚本 `train_seg.py` 已优化为 YOLOv8n-seg（epochs=20, lr0=0.001, 含数据增强）。

同名 PNG 的 SHA256 在两个数据集之间逐张一致（树检测图像为果实分割图像的全图副本）。

### 9.2 已部署权重

| 文件 | 模型 | 大小 | 状态 |
|------|------|------|------|
| `wvcsc_tree_yolov8n.pt` | YOLOv8n Detect | 6.0 MB | ✅ 已训练，已部署 |
| `wvcsc_fruit_yolov8n_seg.pt` | YOLOv8n-seg Seg | — | ❌ 待标注后训练 |

### 9.3 模型路径解析

推理时 `two_stage_yolo` 通过 `resolve_yolo_model_path()` 加载模型：
- 相对路径（如 `wvcsc_tree_yolov8n.pt`）→ 解析到 `<wvcsc_rgb_vision share>/models/` 下
- 绝对路径直接使用
- 配置位于 `wvcsc_rgb_vision/config/vision_sim.yaml` 的 `tree_model_path` / `fruit_model_path`

### 9.4 YOLO 运行时

```bash
# 创建隔离 venv（不修改系统 Python）
mkdir -p /home/robot/venvs
python3 -m venv --system-site-packages /home/robot/venvs/wvcsc_yolo_ros
PYTHONNOUSERSITE=1 /home/robot/venvs/wvcsc_yolo_ros/bin/python -m pip install \
  --upgrade --force-reinstall --no-cache-dir \
  -r /home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_rgb_vision/requirements-yolo-runtime.txt
```

---

## 10. 实机前仍需完成

- C10 实机内参与手眼外参标定
- 相机持久设备路径、格式、曝光、时间戳和断线恢复
- 喷嘴真实工作距离、流量和启停延迟标定
- Alicia-M 关节速度、加速度和扫描范围确认
- 底盘停车误差和树行地图误差统计
- 喷洒互锁、急停、药液状态和人员安全区

---

## 11. 视觉伺服诊断与分析（2026-07-17）

### 11.1 当前问题

最新 rosbag `wvcsc_servo_20260717_2241` 表明视觉控制器已经按约 20 Hz
发布 Twist，但 MoveIt Servo 的 JointTrajectory 只有约 1 Hz：

| 目标 | 对准时间 | 像素误差范数 | Twist | JointTrajectory | 结果 |
|------|----------|--------------|-------|-----------------|------|
| `fruit-1` | 8.05 s | 93.36 → 86.58 px | 19.34 Hz | 1.01 Hz | stalled |
| `fruit-35` | 4.98 s | 175.62 → 173.24 px | 17.94 Hz | 1.02 Hz | stalled |

两个目标均未进入双轴 `±8 px` 稳定区，最终 `sprayed=0`。命令速度积分
只有约 14.6% 转化为相机实际路径，主瓶颈是 MoveIt Servo 输出链，不是
PID 输出方向或视觉目标有效性。

处理方式：

1. MoveGroup 保留 KDL，用于全局规划；
2. Servo 节点不加载 `robot_description_kinematics`，改用逆雅可比；
3. `use_gazebo=false`，取消每条命令 30 点、1.5 秒的旧 Gazebo 兼容轨迹；
4. `publish_joint_velocities=false`，匹配位置型 `arm_controller`；
5. PID、速度、碰撞和奇异点参数暂不继续放大。

### 11.2 已有调试能力

| 通道 | 说明 |
|------|------|
| `/vision/visual_servo_debug` (5 Hz JSON) | 误差、命令速度、伺服状态、关节角度 |
| `/vision/debug_image` | 树/果检测框 + 瞄准点标注 |
| `/vision/target` (Target2D) | 当前选中目标的像素坐标和置信度 |
| `/servo_node/delta_twist_cmds` | MoveIt Servo 实际执行的 twist |
| `/servo_node/status` | MoveIt Servo 状态码 |
| ROS 日志 WARN | `_abort()` 时输出完整终止诊断 |

### 11.3 rosbag 录制

只使用独立脚本手动录制，不接入 launch，也不恢复离线 CSV/报告工具：

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
bash src/wvcsc_visual_servo/scripts/record_servo_bag.sh
```

任务完成后按 `Ctrl+C` 停止。脚本自动 source ROS 和工作区，默认写入
`~/bags/wvcsc/wvcsc_servo_YYYYMMDD_HHMM`。可通过 `WVCSC_BAG_DIR` 修改输出目录。

`/vision/visual_servo_debug` 是 JSON 格式的 `std_msgs/String`。PlotJuggler
不能直接把 JSON 字段拆成数值曲线，显示为乱码不代表 bag 损坏；后续将 bag
交给 Codex 直接反序列化分析。

### 11.4 Fairino 视觉伺服参考边界

可参考 `/home/robot/fairino_robotarm/src/visual_servo` 中的真实控制周期、目标
过期零速度、短时预测、跳变限制、速度/加速度限制、Servo 状态策略和稳定
handoff。当前 WVCSC 已采用其中大部分基础思想。

现阶段不移植 LADRC、NLADRC、MPC、Adaptive PID、三维深度控制、Fairino
专用 DH 模型、250 Hz 参数或其面向硬件的安全配置。Fairino 使用带深度的
基坐标系三维误差，WVCSC 当前只做固定喷洒距离下的图像平面 XY 对准，参数
不能直接照搬。本轮关闭 Servo 在线碰撞检查仅是 Gazebo 吞吐修复，不是
Fairino 参数移植，真实机械臂必须重新启用并验证。

只有在 JointTrajectory 达到 `≥18 Hz` 后仍存在振荡、噪声或明显延迟，才
根据新 rosbag 选择性增加目标跳变限制、延迟补偿或前馈。

### 11.5 验收门槛

- `/servo_node/delta_twist_cmds` 和 `/arm_controller/joint_trajectory` 均 `≥18 Hz`
- 每条 JointTrajectory 只有一个 `time_from_start=0.05 s` 的目标点
- 相机实际路径/命令积分比例 `≥60%`
- 双轴误差均 `≤4 px` 且连续保持 `≥0.5 s`
- 中位收敛时间 `≤5 s`，最大 `≤8 s`
- 连续三轮满足 `detected == sprayed`、`unresolved=0`、`alignment_failures=0`

### 11.6 2026-07-18 吞吐、检测去重与人工复位修复

`wvcsc_servo_20260718_1344` 中视觉输入稳定，但非零 Twist 共 77 条时
JointTrajectory 只有 5 条。Servo 状态始终为 0，碰撞缩放始终为 1.0，
而碰撞检查实际仅约 2.18 Hz，说明 Humble Servo 的同步碰撞回调长期占用
默认互斥回调组，阻塞了 Twist 和 stop 服务。

本轮已实施：

- Gazebo 配置将 Servo 内部 `check_collisions` 设为 `false`；观察位姿、
  重心位姿的碰撞 IK、MoveIt 轨迹碰撞规划、关节限位和奇异点保护仍保留。
  该配置不得直接用于真实机械臂。
- 对齐结束先按 20 Hz 连续发布 0.25 秒零 Twist，再且仅调用一次 stop 服务；
  日志分别记录原始对齐结果、stop 往返时间和最新目标年龄。
- 最终容差改为双轴 `≤4 px` 并保持 `≥0.5 s`，近目标速度缩放改为 `1.0`，
  对齐超时统一为 8 秒。
- 果实模型显式使用 `iou=0.45`；跟踪前按 `IoU≥0.35` 或中心距离 `≤10 px`
  去重。同位置跨类别且置信度差 `<0.10` 的结果视为歧义，不进入喷洒队列。
- 调试标签简化为 `目标ID + 类别 + 置信度`，选中目标单独高亮；任务侧再次
  防御性去重，避免一颗病果进入队列多次。

机械臂因安全故障进入 stop 锁定后，只允许人工确认恢复：

```bash
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: reset}"
```

必须等待：

```text
Reset reached HOME; send resume to unlock motion
```

再依次执行：

```bash
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: resume}"

ros2 service call /mission/reset std_srvs/srv/Trigger "{}"
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

若 HOME 复位失败，保持锁定，禁止发送 `resume` 或重新启动任务。下一轮仍用
`record_servo_bag.sh` 手动录包；只有 JointTrajectory 与非零 Twist 均达到
`≥18 Hz` 后，才根据数据决定是否继续整定 PID。

### 11.7 2026-07-18 增益补偿与目标可靠性修复

基线 `wvcsc_servo_20260718_1421` 仍未完成喷洒：

| 目标 | 有效视觉帧 | 误差范数 | 结果 |
|------|-------------|----------|------|
| `fruit-1` | 100% | 18.17 → 5.61 px | 7.69 s 后停滞 |
| `fruit-14` | 100% | 33.20 → 11.31 px | 8.28 s 超时 |
| `fruit-42` | 7.7% | 40.65 → 40.79 px | 持续丢失后失败 |

本轮代码变更：

- Gazebo 视觉伺服 `Kp` 从 `0.45` 提高到 `1.00`，`Kd=0.005`、速度
  `0.08 m/s`、加速度 `0.60 m/s²` 和 20 Hz 周期保持不变；停滞有效改善量
  从 4 px 调整为 1 px。
- 连续目标失效门槛改为 `0.75 s`。无效期间立即发零速、不累计停滞时间；
  目标恢复后重新开始进展窗口，并清空旧预测速度。
- 超时、停滞和目标丢失均冻结 stop 前最后有效目标，Action 结果和 debug
  保留最后有效像素误差、连续不可用时长，不再被 stop 阶段的消息覆盖。
- YOLO 推理阈值继续保持 `0.10`，供跟踪使用；任务队列和重心后 Servo
  准入阈值提高到 `0.30`。
- 重心后必须连续 `0.50 s` 同时满足 `Target2D.valid=true`、置信度
  `≥0.30`、双轴误差 `≤48 px`，才允许发送 Align Goal。门控失败直接切换
  下一观察位，不消耗视觉对准尝试。

代码侧验证：

```text
93 passed
wvcsc_visual_servo: build passed
wvcsc_arm_task: build passed
git diff --check: passed
```

Gazebo 尚未复验，因此不能把本轮标记为“快速收敛已完成”。下一轮执行：

```bash
cd ~/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select wvcsc_visual_servo wvcsc_arm_task
source install/setup.bash

# 另开终端手动录包
bash src/wvcsc_visual_servo/scripts/record_servo_bag.sh
```

复验必须同时满足 11.5 节门槛，并重点确认：

- 低置信度目标不再进入 Align Action；
- 进入 Align 的目标有效率 `≥95%`，连续丢失不超过 `0.25 s`；
- `fruit-1`、`fruit-14` 双轴均进入 `±4 px` 并保持 `≥0.5 s`；
- 没有因 `min_progress_px=1.0` 产生虚假停滞；
- `sprayed == unique_diseased_fruits`、`unresolved=0`。

若控制链达标但重心后识别率仍不足，应优先补充“果实由画面上方移动到中心”
的 orchard seed 隔离数据并重训，而不是继续提高 PID。新权重启用门槛为：
重心后目标有效率 `≥95%`、病果置信度中位数 `≥0.30`、未见 seed 病果
recall `≥0.90`。

# WVCSC ARMSpray 视觉喷洒闭环实施方案

> 更新日期：2026-07-17
> 当前阶段：两级 YOLO 作业链路已编码完成，mock 模式状态机通过；下一步目标是在真实 YOLO 权重训练完成后完成完整 ARM_SPRAYING 七阶段闭环验收。
> 验证结果：16 个相关包构建通过，74 项针对性测试通过。动态观察位姿、扇形扫描、病果队列、对准重试和观察距离恢复均已实现。

## 1. 目标与边界

### 1.1 本次目标

完成一棵树的完整视觉喷洒闭环：

```text
MOVING_TO_OBSERVE → SCANNING_TREE → DETECTING_FRUITS → QUEUING
    → ALIGNING → SPRAYING → RETURNING_TO_OBSERVE → 复检直到队列为空 → HOME
```

Mock 模式下已完成状态机验证。真实权重就绪后的验收目标是：四棵树全部按上述链路完成，每棵树最少喷洒 1 个病果，连续运行三轮无安全锁定。

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
YOLOv8s-seg 推理 (tree + healthy_fruit + diseased_fruit)
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
detection_timeout_sec: 2.0     # 最多等 2 秒
fruit_confidence: 0.50         # 病果检测置信度阈值
```

**两类 YOLO 模型的职责分工**：

| 模型 | 任务 | 输入 | 输出 |
|------|------|------|------|
| `wvcsc_tree_yolov8s.pt` | 目标检测 (Detect) | 完整 1280×720 图像 | `tree` bbox |
| `wvcsc_fruit_yolov8s_seg.pt` | 实例分割 (Seg) | 完整图像（tree ROI 由下游过滤） | `healthy_fruit` mask + `diseased_fruit` mask |

### 2.5 [4] QUEUING — 病果队列

```
候选列表 (diseased_fruit 的 Detection2D 消息)
    ↓
spray_task._queue():
    1. 过滤已处理 (processed) 和已耗尽 (exhausted) 的目标
       - IoU ≥ processed_iou_threshold (0.30) → 已处理
       - 中心距离 ≤ processed_center_distance_px (40px) → 已处理
    2. 按距离图像中心的远近排序 (近的优先)
    3. 同距离按下置信度降序
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

### 5.2 Mock 模式启动（无 YOLO 权重回归）

```bash
ros2 launch wvcsc_simulation system_sim.launch.py \
  use_nav2:=true use_rviz:=false \
  use_mock_uav:=true use_replay_uav:=false \
  use_mission_manager:=true auto_start_mission:=true \
  use_web_ui:=false perception_mode:=mock
```

### 5.3 YOLO 模式启动（需权重文件就绪）

```bash
./run_system_sim.sh
# 或手动指定
ros2 launch wvcsc_simulation system_sim.launch.py \
  perception_mode:=yolo \
  yolo_python_executable:=/home/robot/venvs/wvcsc_yolo_ros/bin/python \
  auto_start_mission:=false
```

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

### 6.1 Mock 模式（当前可验收）

- [ ] `/mission/plan` 四个作业位姿依次为 `(3,0.5)` `(5,-0.5)` `(11,0.5)` `(13,-0.5)`
- [ ] Nav2 按输入顺序到达四棵树
- [ ] 每次导航成功后 `/odom` 连续停稳 1 秒才启动机械臂
- [ ] 每棵树的 `tree_hint` 由 TF 动态转换为 `alicia_base_link` 坐标
- [ ] 动态观察位姿规划成功且避免碰撞
- [ ] Mock 模式确认 tree、生成病果队列、完成对准+喷洒+复检
- [ ] 每棵树完成后机械臂回到 HOME
- [ ] 任一故障不进入下一目标
- [ ] 最终 `MISSION_COMPLETED`，`completed_targets=4`
- [ ] 同一启动方式连续完成 3 轮

### 6.2 YOLO 模式（权重就绪后）

- [ ] `perception_mode:=yolo` 启动不报错
- [ ] SCANNING_TREE 阶段真实 YOLO 检测到 tree（conf ≥ 0.50）
- [ ] DETECTING_FRUITS 阶段真实 YOLO Seg 检测到 diseased_fruit
- [ ] 病果队列按距离排序正确
- [ ] 每个病果的 IBVS 在 8s 内对准（fine_tolerance_px=8, stable_frames=10）
- [ ] Spray Action 调用成功，sprayed 计数正确
- [ ] 复检逻辑正确（喷洒后回到观察姿态重新检测）
- [ ] 无 Servo 安全锁定或碰撞
- [ ] 最终 `/mission/status` 显示 `MISSION_COMPLETED`
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
| 第一级 | 果树检测 | YOLOv8s Detect | `tree` (0) |
| 第二级 | 果实实例分割 | YOLOv8s-seg Seg | `healthy_fruit` (1), `diseased_fruit` (2) |

分离的理由：
- tree Detect：快速判断果树是否在视野中（不需要 mask）
- fruit Seg：精确的像素级分割用于 IBVS 对准（需要 mask 质心）

### 8.4 单目 RGB 边界

- IBVS 只修正图像平面 X/Y，不发 Z 轴速度
- 喷洒距离由停车位姿和观察距离参数保证（不虚构单目深度）
- 仅当 Servo 正常、目标可信、喷洒就绪时开阀

---

## 9. 数据集与模型

### 9.1 数据采集

已于 2026-07-16 重新采集 30 张无标注 C10 模拟图像：

- 6 个视角 × 5 个 seed = 30 张，`1280×720` PNG
- 训练集：seed 50-53 (24 张)，验证集：seed 54 (6 张)
- 每个视角来自 `camera_look_at_pose` 的真实观察位姿
- 同名 PNG 的 SHA256 在两个数据集之间逐张一致
- 标签目录已创建但为空（待人工标注）

### 9.2 已部署权重

| 文件 | SHA256 |
|------|--------|
| `wvcsc_tree_yolov8s.pt` | `71396df53b2ba831ac8380e70c64593967c18736efd63c3bfd8dbd6c39c9c6af` |
| `wvcsc_fruit_yolov8s_seg.pt` | `1eb52a516227a74f4be59f7352e701ae3f6510a86891428907c24d8506d8503a` |

### 9.3 YOLO 运行时

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

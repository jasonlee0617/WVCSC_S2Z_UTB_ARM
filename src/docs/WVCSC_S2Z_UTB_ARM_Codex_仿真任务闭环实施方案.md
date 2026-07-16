# WVCSC 空地协同喷洒仿真闭环实施方案

> 更新日期：2026-07-16
> 当前阶段：已接入两级 YOLO 作业链路、独立 Motion Control 与逐病果喷洒状态机；默认 `perception_mode:=mock` 用于无权重回归，配置权重后切换为 `yolo`。
> 验证结果：16 个相关包构建通过，74 项针对性测试通过。`perception_mode:=mock` 已覆盖同一接口的状态机回归；真实权重的 Gazebo 图像闭环需在模型训练完成后单独验收。

## 1. 已确认的作业目标

四棵目标按无人机给出的顺序逐棵处理：

```text
tree_01 left → tree_02 right → tree_03 left → tree_04 right
```

当前闭环：

```text
Mock UAV 发布病树 map 坐标
  → 任务管理器生成道路作业位姿
  → Nav2 导航
  → /odom 连续停稳确认
  → 机械臂到左/右观察姿态
  → 逐病果对准与 Spray Action
  → 机械臂返回 HOME
  → 下一棵树
```

未来视觉闭环：在每棵树上识别全部 `diseased_fruit`，去重后逐个完成图像平面 XY 对准和喷洒，全部处理完才返回 HOME。

## 2. 当前阶段的关键决策

### 2.1 无人机只发送树的全局信息

`DiseaseTree` 继续提供：

- `tree_id`；
- `map` 坐标；
- 置信度；
- `spray_side`；
- 当前仿真使用的喷洒时长；
- 可选证据 URI。

不让无人机发送“相对小车坐标”。小车移动后该相对量会失效。未来需要树相对小车的方向时，由任务端用最新 TF 将树的 `map` 坐标变换到 `base_footprint`。

### 2.2 导航到道路作业位姿，不导航到树干中心

目标树本身是障碍物，Nav2 不能把树干中心作为底盘中心目标。当前道路为直线，作业位姿统一为：

```text
goal_x   = tree.position.x
goal_y   = road_center_y + 0.5  # left
         = road_center_y - 0.5  # right
goal_yaw = road_yaw
```

默认 `road_center_y=0.0`、`road_yaw=0.0`、`docking_lateral_offset=0.5`，所以四个作业位姿为：

```text
tree_01 → ( 3.0,  0.5, 0.0)
tree_02 → ( 5.0, -0.5, 0.0)
tree_03 → (11.0,  0.5, 0.0)
tree_04 → (13.0, -0.5, 0.0)
```

已去除 `standoff_distance` / `spray_standoff_distance`。`docking_lateral_offset` 只表示小车从道路中心向作业侧靠近的距离，不代表喷嘴到病斑的物理喷距。后续若道路不规则，应由地图或路径生成器提供道路中心线/候选作业位姿。

### 2.3 去除独立病斑模型

`orchard.world` 不再加载四个悬空的病斑模型。果树网格上的红色果实标注为 `healthy_fruit`，黄色果实标注为 `diseased_fruit`，分布由 `orchard_seed` 可复现地生成。

### 2.4 感知模式与独立运动控制

`system_sim.launch.py` 默认：

```text
perception_mode=mock
```

`mock` 发布与两级 YOLO 相同的 tree、病果和选中目标接口，用于状态机回归；`yolo` 加载 tree Detect 与 fruit Seg 权重。MoveIt Servo 与 Spray Action 在启用机械臂控制时始终启动，不再提供绕过对准或喷洒执行器的开关。

`wvcsc_motion_control` 保持独立节点和 setup 入口，是 `/motion_control/command` 的唯一解释者，并以锁存 `/motion_control/locked` 向喷洒任务广播锁状态。

## 3. 当前可执行状态机

每棵树必须按以下顺序执行：

```text
NAVIGATING
  → VERIFYING_STOP
  → ARM_SPRAYING / MOVING_TO_OBSERVE
  → SCANNING_TREE
  → DETECTING_FRUITS / QUEUING
  → ALIGNING / SPRAYING / RETURNING_TO_OBSERVE
  → RETURNING_HOME
  → TARGET_COMPLETED
```

任务完成：

```text
MISSION_COMPLETED
completed_targets=4
skipped_targets=0
nav_goal_active=false
arm_goal_active=false
```

当前一次 `/arm/execute_spray` Goal 的边界仍是“一棵树”。其内部执行：

```text
动态 tree_hint 观察位姿 → tree 确认 → 病果队列 → XY 对准 → Spray Action → 复检 → HOME
```

单目 RGB 仍只提供像素、掩膜和置信度；喷距由停车位与观察位姿保证，不虚构深度闭环。

## 4. YOLO 接入后的推荐作业流程

YOLO 类别固定为：

```text
0: tree
1: healthy_fruit
2: diseased_fruit
```

未来仍保持“一棵树一个 ExecuteSpray Goal”，将多病斑循环封装在机械臂/视觉作业内部，任务管理器不需要知道像素级病斑数量：

```text
1. 小车到道路作业位姿并停稳
2. 用 TF 计算目标树在 base_footprint 下的方位
3. MoveIt 规划到该侧的粗观察姿态
4. 启用 YOLO 推理，先锁定当前 tree 区域
5. 未找到树时，在关节和碰撞约束内做有限扇形搜索
6. 只接收当前 tree ROI 内的 healthy_fruit / diseased_fruit
7. 对检测结果跟踪、去重并建立待喷洒队列
8. 逐个处理待喷洒目标：
   8.1 锁定一个 diseased_fruit
   8.2 仅做图像平面 XY 视觉伺服
   8.3 连续稳定若干帧后停止 Servo
   8.4 开启喷洒并计时
   8.5 关闭喷洒并标记该目标已处理
   8.6 返回完全相同的观察姿态，重新检测并继续下一个未处理目标
9. 扫描完成后返回 HOME
10. 任务管理器进入下一棵树
```

相机建议保持持续采集，只按任务状态启停 YOLO 推理，不频繁关闭和重启相机设备。

### 4.1 目标关联与去重

必须先选中 `tree`，再处理其内部的果实，避免把相邻树的果实归到当前任务。队列只加入 `diseased_fruit`，至少需要：

- `track_id`；
- 包围框中心和尺寸；
- 检测置信度；
- 最近更新时间；
- `PENDING / ALIGNING / TREATED / FAILED` 状态。

同一目标不能因连续多帧检测被重复喷洒。第一版可用包围框中心距离或 IoU 做短时关联，暂不增加复杂三维重建。

### 4.2 扇形搜索

不建议直接向底层关节发送“J1 固定转多少度”。推荐由 MoveIt 执行受限观察姿态或小步扫描：

- 左/右侧决定搜索中心；
- J1 只在配置的安全区间内往返；
- 每个扫描点等待图像稳定；
- 命中目标后立即停止扫描；
- 超过最大角度、最大时间或最大扫描次数后结束本树任务。

观察位姿由 `tree_hint → camera look-at pose → tool0 pose` 动态计算，不再维护固定左右关节姿态回退。

### 4.3 视觉伺服边界

第一版只做图像平面 XY 对准是合理的，但它只能解决“目标位于画面中心”，不能证明实际喷距安全。因此：

- 光轴 Z 命令保持为零；
- 不用单目 RGB 虚构深度；
- 只有底盘停稳、目标可信、Servo 正常且喷洒器就绪时才能喷洒；
- 对准成功后先发零速并确认 Servo 停止，再开启喷洒；
- 目标丢失、奇异、碰撞、超限或通信过期时立即关闭喷洒。

真实设备最终仍需要喷嘴工作距离约束，来源可为标定后的停车位姿、深度传感器或测距传感器；此项暂缓，不在当前仿真中硬编码。

### 4.4 推荐异常语义

| 情况 | 推荐行为 |
|---|---|
| 未找到 `tree` | 有限扫描后记录 `TREE_NOT_FOUND`，不喷洒，返回 HOME |
| 找到树但没有 `diseased_fruit` | 标记 `INSPECTED_NO_DISEASE`，不喷洒并返回 HOME |
| 单个病斑对准失败 | 关闭喷洒，记录该病斑失败；第一版建议结束当前树并 HOME |
| 单个病斑喷洒成功 | 标记 `TREATED`，继续该树剩余目标 |
| Servo/碰撞/奇异安全故障 | 立即停机并锁定，整项任务失败 |
| HOME 失败 | 保持锁定，不进入下一棵树 |

`ExecuteSpray` 已增加 `INSPECTED_NO_DISEASE`；没有病果属于检查成功，不作为任务失败或跳过。

## 5. 当前启动与验收

2026-07-15 已完成无 GUI 的导航、观察、喷洒接口与 HOME 基线验收。两级 YOLO 权重的真实图像闭环仍需在人工标注和训练完成后验收。

### 5.1 构建

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to wvcsc_simulation
source install/setup.bash
```

### 5.2 启动当前四树闭环

```bash
ros2 launch wvcsc_simulation system_sim.launch.py \
  use_nav2:=true use_rviz:=false \
  use_mock_uav:=true use_replay_uav:=false \
  use_mission_manager:=true auto_start_mission:=true \
  use_web_ui:=false perception_mode:=mock
```

### 5.3 监控

```bash
ros2 topic echo --no-daemon \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /mission/status
```

```bash
ros2 topic echo --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /mission/plan
```

```bash
ros2 action list | grep -E 'navigate_to_pose|execute_spray'
ros2 topic hz /odom
```

### 5.4 当前阶段验收标准

- [ ] `/mission/plan` 的四个作业位姿依次为 `(3,0.5)`、`(5,-0.5)`、`(11,0.5)`、`(13,-0.5)`；
- [ ] Nav2 按输入顺序到达四个作业位姿；
- [ ] 每次导航成功后 `/odom` 连续停稳 1 秒才启动机械臂；
- [ ] 每棵树由 `tree_hint` 生成可达的动态观察姿态；
- [ ] Mock 模式下确认 tree、生成病果队列，并完成对准、喷洒、复检；
- [ ] 每棵树完成后机械臂均回到 HOME；
- [ ] 任一导航或机械臂失败时不进入下一目标；
- [ ] 最终 `MISSION_COMPLETED`，完成数为 4；
- [ ] 同一启动方式连续完成 3 轮。

真实 YOLO 模式还需验证 tree ROI、果实实例掩膜、像素误差和视觉 Servo 状态。

### 5.5 Nav2 Qt 手动单点/多点作业

手动任务模式与 Mock/Replay UAV 互斥。启动时设置
`use_nav2_qt:=true` 后，系统不会启动 Mock 或 Replay UAV，且必须保持
`auto_start_mission:=false`：

```bash
ros2 launch wvcsc_simulation system_sim.launch.py \
  use_rviz:=true use_nav2_qt:=true \
  use_mock_uav:=false use_replay_uav:=false \
  auto_start_mission:=false
```

1. RViz 点击 `2D Estimate Pose`，Qt 点击“记录起点”；Qt优先记录
   `/initialpose`，没有该消息时回退读取 `map -> base_footprint`。
2. RViz 点击 `2D Goal Pose`。此工具只发布 `/manual_goal_pose`，不会直接发起
   Nav2 Action。
3. 每次RViz选点后点击“添加终点到列表”，检查顺序和左/右喷洒侧别。
4. 列表恰有一个终点时可点击“单点导航+喷洒”；喷洒成功后该终点自动删除。
   列表有两个及以上终点时可点击“多点导航+喷洒”；多点完成后列表保留。
5. 任务执行期间可使用暂停、继续、跳过当前、取消和返回起点；最终查看
   `/mission/status` 的 `MISSION_COMPLETED` 与完成数量。

Qt向 `/mission/load_manual` 提交精确停车位姿；手动目标不再使用树坐标的
`0.5m` 横向停靠偏移。实机使用同一个GUI启动文件，只需在已启动实机底盘、Nav2、
任务管理器后执行：

```bash
ros2 launch my_navigation2 nav2_qt.launch.py use_sim_time:=false
```

## 6. 两级 YOLO 数据与后续实施

原始 C10 种子集位于 `/home/robot/ultralytics-main/datasets/wvcsc_fruit_seg/`。当前已完成 30 张 `1280×720` 图像及实例分割标注：`train` 24 张、`val` 6 张；采集来源为 `/camera/camera/color/image_raw`。

本轮使用 seed `50–54`，每个 seed 采集左侧 3 棵、右侧 3 棵；果树资产固定每棵最多 5 个果实，病果按 seed 固定随机生成 1–2 个，健康果为鲜红色、病果为黄色。采集前临时隐藏 tool0 白球，采集后已恢复原始 URDF，文件 SHA256 为 `71b93d066b6d7c6354d5412da4e37352518f270a4ff77161feb192e2b813e5a5`。

数据采集验收命令：

```bash
PYTHONPATH=$PWD/wvcsc_simulation python3 -c \
  "from wvcsc_simulation.yolo_seed_dataset import validate_fruit_seg_dataset; \
   print(validate_fruit_seg_dataset('/home/robot/ultralytics-main/datasets/wvcsc_fruit_seg'))"
```

注意：Gazebo 11 相机需要在 source ROS 2 后再次加载 Gazebo 环境，避免 `GAZEBO_RESOURCE_PATH` 被 ROS 环境覆盖：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:/opt/ros/humble/share
```

采集工具在机械臂成功进入 `SCANNING_TREE` 的稳定观察姿态后保存一帧。类别规范为 `tree`、`healthy_fruit`、`diseased_fruit`，后续人工标注和训练工程放在 `/home/robot/ultralytics-main/datasets`：第一级完整图训练 `tree` Detect，第二级树冠 ROI 训练健康/病害果实实例分割。观察距离默认为 `1.40m`，位于已确认的 `0.8–1.5m` 喷距范围内，并为左右两侧 MoveIt 规划保留更多可达空间。

单树 Action 的目标状态机是：

```text
MOVING_TO_OBSERVE
→ SCANNING_TREE
→ DETECTING_FRUITS
→ QUEUING
→ ALIGNING
→ SPRAYING
→ RETURNING_TO_OBSERVE
→ DETECTING_FRUITS / RETURNING_HOME
```

Action 已实际执行上述状态。当前候选权重已安装到 `wvcsc_rgb_vision/models/wvcsc_fruit_yolov8n_seg.pt`，SHA256 为 `a882588ceb56d22d4f1237db0f505acfcae6123dc4cc5988d48cdfdd09b59913`。该模型的 Mask mAP50 为 `0.196`、病果 Mask mAP50 为 `0.135`，`conf=0.50` 时 6 张 val 图像均无检测，因此只用于 ROS 接口接线验证，不作为可用闭环模型。

仿真配置临时使用 `assume_tree_in_view:=true`：观察位姿已将单棵树置于画面中，视觉节点以完整画面作为 tree ROI，同时继续发布树确认消息。这只是临时仿真边界，合格 tree 模型就绪后必须设回 `false`。常规回归仍使用 `perception_mode:=mock`；只做 YOLO 接线验证时执行：

```bash
ros2 launch wvcsc_simulation system_sim.launch.py \
  perception_mode:=yolo auto_start_mission:=false
```

重训后只有在病果 Mask precision、recall 均 `≥0.80`、Mask mAP50 `≥0.70`，且 `conf=0.50` 时至少检出 val 中 `4/5` 个病果，才允许进入自动任务验收。真实图像验收还需覆盖实例掩膜安全点、目标 ID 连续性、稳定帧对准、目标丢失、超时、Servo 安全锁定与逐病果复检。

## 7. 保留的接口边界

| 接口 | 当前职责 |
|---|---|
| `/uav/disease_trees` | 发布树级任务，不发布机械臂控制量 |
| `/mission/plan` | 发布树坐标与道路作业位姿 |
| `/navigate_to_pose` | 到达道路作业位姿 |
| `/arm/execute_spray` | 完成一棵树的观察、处理和 HOME |
| `/mission/status` | 发布任务、目标计数和活动 Goal 状态 |
| `/camera/camera/color/image_raw` | 仿真/实机统一 RGB 图像 |
| `/vision/tree_detections` | tree 候选；临时仿真模式为完整画面 ROI |
| `/vision/fruit_detections` | YOLOv8n-seg 健康/病果实例候选 |
| `/vision/selected_target_id` | Arm Task 选择的病果实例 ID |
| `/vision/target` | 已选病果的掩膜安全点，供 IBVS 使用 |
| `/vision/align_target` | 对指定 target_id 执行单个目标的 XY 对准 |
| `/motion_control/locked` | 独立 Motion Control 广播的权威运动锁 |

检测结果使用标准 `Detection2DArray`，不新增病果队列消息；队列仅在 Arm Task 内存中维护。

## 8. 实机前仍需完成

- C10 实机内参与手眼外参标定；
- 相机持久设备路径、格式、曝光、时间戳和断线恢复测试；
- 喷嘴真实工作距离、流量和启停延迟标定；
- Alicia-M 关节速度、加速度和允许扫描范围确认；
- 底盘停车误差和树行地图误差统计；
- 喷洒互锁、急停、药液状态和人员安全区验证。

## 9. 当前完成边界

当前应完成的是：

```text
四树坐标任务
→ 道路作业位姿导航
→ 停稳
→ 动态观察位姿
→ tree Detect / fruit Seg
→ 病果队列与逐个 XY 伺服
→ Spray Action 与复检
→ HOME
→ 下一树
```

训练权重、真实 C10 标定和真实喷洒硬件仍是下一阶段工作。
ARM_SPRAYING (一棵树开始)
  │
  ├─ [1] MOVING_TO_OBSERVE    → 机械臂到达左/右粗观察姿态
  ├─ [2] SCANNING_TREE        → 启用YOLO tree类别推理，确认果树在主视野
  ├─ [3] DETECTING_FRUITS     → 在tree ROI内执行果实实例分割
  ├─ [4] QUEUING              → 对检测结果去重、排序、建立待喷洒队列
  ├─ [5] ALIGNING             → IBVS逐个对准病斑（图像平面XY）
  ├─ [6] SPRAYING             → 对准→稳定→开喷洒→计时→关喷洒
  ├─ [7] RETURNING_TO_OBSERVE / HOME → 复检下一病果或全部完成返回HOME

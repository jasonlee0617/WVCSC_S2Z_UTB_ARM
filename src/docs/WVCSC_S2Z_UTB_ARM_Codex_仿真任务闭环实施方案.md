# WVCSC 空地协同喷洒仿真闭环实施方案

> 更新日期：2026-07-15  
> 当前阶段：先完成不依赖视觉模型的四树导航、观察、模拟喷洒和 HOME 闭环。YOLO 与视觉伺服待模型训练完成后接入。
> 验证结果：相关包构建通过，96 项测试通过；采用左右 `0.5m` 停靠偏移的真实 `gzserver + Nav2 + MoveIt` 无 GUI 闭环已完成 1 轮四目标，最终 `MISSION_COMPLETED`、`completed_targets=4`、无活动 Goal。

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
  → 定时模拟一次整树喷洒
  → 机械臂返回 HOME
  → 下一棵树
```

未来视觉闭环：在每棵树上识别全部 `disease_patch`，去重后逐个完成图像平面 XY 对准和喷洒，全部处理完才返回 HOME。

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

`orchard.world` 不再加载四个悬空的 `disease_patch` 模型，相应 Gazebo 模型资源已删除。未来训练和仿真验证时，将果树模型上的深色果实临时标注为 `disease_patch`。

### 2.4 当前默认不启动视觉闭环

`system_sim.launch.py` 默认：

```text
use_color_vision=false
use_vision_alignment=false
use_spray_action=false
```

相机传感器仍发布图像，但 Gazebo 中不显示蓝色相机视锥。颜色分割和 MoveIt Servo 代码保留，现阶段不作为四树闭环的通过条件。

## 3. 当前可执行状态机

每棵树必须按以下顺序执行：

```text
NAVIGATING
  → VERIFYING_STOP
  → ARM_SPRAYING / MOVING_TO_OBSERVE
  → SPRAYING
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

当前一次 `/arm/execute_spray` Goal 的边界是“一棵树”。视觉未接入时，其内部执行：

```text
左/右固定观察姿态 → 定时模拟喷洒 → HOME
```

这只是任务编排占位，不代表已实现真实病虫害定位和精准喷洒。

## 4. YOLO 接入后的推荐作业流程

YOLO 类别固定为：

```text
tree
disease_patch
```

未来仍保持“一棵树一个 ExecuteSpray Goal”，将多病斑循环封装在机械臂/视觉作业内部，任务管理器不需要知道像素级病斑数量：

```text
1. 小车到道路作业位姿并停稳
2. 用 TF 计算目标树在 base_footprint 下的方位
3. MoveIt 规划到该侧的粗观察姿态
4. 启用 YOLO 推理，先锁定当前 tree 区域
5. 未找到树时，在关节和碰撞约束内做有限扇形搜索
6. 只接收当前 tree ROI 内的 disease_patch
7. 对检测结果跟踪、去重并建立待喷洒队列
8. 逐个处理待喷洒目标：
   8.1 锁定一个 disease_patch
   8.2 仅做图像平面 XY 视觉伺服
   8.3 连续稳定若干帧后停止 Servo
   8.4 开启喷洒并计时
   8.5 关闭喷洒并标记该目标已处理
   8.6 重新检测，继续下一个未处理目标
9. 扫描完成后返回 HOME
10. 任务管理器进入下一棵树
```

相机建议保持持续采集，只按任务状态启停 YOLO 推理，不频繁关闭和重启相机设备。

### 4.1 目标关联与去重

必须先选中 `tree`，再处理其内部的 `disease_patch`，避免把相邻树的深色果实归到当前任务。病斑队列至少需要：

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

当前 `observe_left_pose` 和 `observe_right_pose` 可作为粗观察起点，后续再根据相机实际视场调整。

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
| 找到树但没有 `disease_patch` | 标记 `INSPECTED_NO_DISEASE`，返回 HOME |
| 单个病斑对准失败 | 关闭喷洒，记录该病斑失败；第一版建议结束当前树并 HOME |
| 单个病斑喷洒成功 | 标记 `TREATED`，继续该树剩余目标 |
| Servo/碰撞/奇异安全故障 | 立即停机并锁定，整项任务失败 |
| HOME 失败 | 保持锁定，不进入下一棵树 |

在视觉模型接入前，不扩展消息和 Action 错误码；先用现有接口完成可验证闭环，模型行为稳定后再根据真实缺口增加状态。

## 5. 当前启动与验收

2026-07-15 已执行一次真实无 GUI 集成验收：四棵树均完成导航、观察、占位喷洒和 HOME，任务无跳过、无错误，关闭后无遗留仿真进程。GUI 展示和“连续 3 轮”仍需手动验收。

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
  use_web_ui:=false use_mock_vision:=false use_color_vision:=false \
  use_vision_alignment:=false use_spray_simulator:=false \
  use_spray_action:=false
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
- [ ] 左侧树进入左观察姿态，右侧树进入右观察姿态；
- [ ] 每棵树只触发一次当前占位喷洒序列；
- [ ] 每次占位喷洒后机械臂均回到 HOME；
- [ ] 任一导航或机械臂失败时不进入下一目标；
- [ ] 最终 `MISSION_COMPLETED`，完成数为 4；
- [ ] 同一启动方式连续完成 3 轮。

相机检测、病斑数量、像素误差和视觉 Servo 状态不是当前阶段验收项。

## 6. YOLO 模型准备完成后的实施顺序

1. 固化 YOLO 输出契约：类别、置信度、BBox、时间戳和图像坐标系；
2. 用 `tree` ROI 过滤 `disease_patch`，先做离线图片/视频测试；
3. 在 Gazebo 只实现“识别 + 可视化”，不控制机械臂；
4. 增加 TF 方位计算和有限扇形搜索；
5. 增加病斑跟踪、去重和逐个任务队列；
6. 接入 XY 视觉伺服，喷洒器仍关闭；
7. 验证稳定帧、目标丢失、超时、碰撞和奇异保护；
8. 最后开启模拟喷洒，完成一树多目标闭环；
9. 再扩展到四树连续闭环和真实相机标定。

## 7. 保留的接口边界

| 接口 | 当前职责 |
|---|---|
| `/uav/disease_trees` | 发布树级任务，不发布机械臂控制量 |
| `/mission/plan` | 发布树坐标与道路作业位姿 |
| `/navigate_to_pose` | 到达道路作业位姿 |
| `/arm/execute_spray` | 完成一棵树的观察、处理和 HOME |
| `/mission/status` | 发布任务、目标计数和活动 Goal 状态 |
| `/camera/camera/color/image_rect_raw` | 仿真/实机统一 RGB 图像 |
| `/vision/pest_detections` | 未来发布所有 `tree` / `disease_patch` 候选 |
| `/vision/align_target` | 未来执行单个目标的 XY 对准 |

现阶段不新增 YOLO 专用包、病斑队列消息或三维定位接口，避免模型尚未确定时提前固化错误设计。

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
→ 左/右固定观察
→ 一树一次占位喷洒
→ HOME
→ 下一树
```

以下内容明确延后：

```text
YOLO tree/disease_patch 推理
→ 树 ROI 关联
→ 有限扇形搜索
→ 病斑跟踪去重
→ 逐个病斑 XY 视觉伺服
→ 逐个精准喷洒
```

这种拆分保留了最终“一棵树全部病斑逐个喷洒”的目标，同时让当前导航与任务编排可以独立验收。

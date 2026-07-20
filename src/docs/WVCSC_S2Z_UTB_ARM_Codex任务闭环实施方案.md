# WVCSC 空地协同导航、视觉伺服与精准喷洒闭环实施方案

> 更新日期：2026-07-21
> 当前状态：仿真主闭环已验证；实机 Bringup、逐树实测停靠、喷嘴投影补偿和 Alicia-M 自动手眼标定代码已接入。真实内参、手眼、喷嘴、干喷、湿喷和四树连续作业仍必须在现场验收，不能用代码测试替代。

## 1. 最终目标与系统边界

作业闭环为：

```text
建图并保存地图
→ 逐树实测 docking_pose 与 tree_hint
→ AMCL + Nav2 单目标直达停靠
→ 停稳、定位协方差和实际停靠误差门控
→ Alicia-M 移动 C10 到安全观察位
→ tree 检测 + disease_leaf 分割
→ 目标重心
→ 按喷嘴投影点执行图像视觉伺服
→ 复核固定工距、目标和安全状态
→ 喷洒 5 s
→ HOME
→ 下一棵树
```

仿真与实机保持相同的 ROS 接口，但配置、类别和权重严格隔离：

| 项目 | 仿真 | 实机 |
|---|---|---|
| 目标类别 | `diseased_fruit` | `disease_leaf` |
| 目标 ID | `fruit-*` | `leaf-*` |
| 树模型 | `yolov8s_sim.pt` | `yolov8s_real.pt` |
| 分割模型 | `yolov8s_seg_sim.pt` | `yolov8s_seg_real.pt` |
| 模型契约 | 仿真果实 | `detect {0: tree}`、`segment {0: disease_leaf}` |
| Servo 碰撞检查 | 仿真可关闭 | 实机必须开启 |
| 任务输入 | Mock UAV | 默认逐树实测任务 |

实机权重缺失或类别不匹配时必须启动失败，不允许回退到仿真权重。历史旧权重
`wvcsc_fruit_yolov8s_seg1.pt` 和 `wvcsc_tree_yolov8s1.pt` 已退出部署，不再恢复。

## 2. 实机 Bringup 与逐树停靠

顶层入口：

```bash
ros2 launch wvcsc_bringup system_real.launch.py \
  mode:=localization operation:=mission mission_source:=measured \
  map:=/absolute/path/map_new.yaml \
  mission_file:=~/.ros/wvcsc_sites/corn_site.yaml
```

模式互斥：

```text
mode=mapping
  → real_cartographer.launch.py
  → 底盘、LiDAR、IMU、EKF、Cartographer、RViz

mode=localization operation=survey
  → 实机传感器、EKF、AMCL、Nav2、安全门控
  → 不启动机械臂、YOLO、任务管理器

mode=localization operation=mission mission_source=measured
  → 完整实机闭环
  → 加载 ~/.ros/wvcsc_sites/corn_site.yaml
  → 不启动 Mock UAV
```

每棵树显式保存 `map` 坐标系下的 `docking_pose` 和树根 `tree_hint`。地图 YAML 与图片 SHA256 写入任务文件；地图改变后旧任务必须重新校验或采集。Nav2 返回成功后仍需满足：

- `/ekf_odom` 连续停稳 1 s；
- `/amcl_pose` 未过期且协方差合格；
- 实际位置误差不超过 0.12 m；
- 实际航向误差不超过 0.12 rad；
- 第一次超限只允许重发同一停靠点一次；第二次失败时禁止启动机械臂。

## 3. 标定产物及依赖顺序

固定安装 C10 和喷嘴后，严格按以下顺序执行：

```text
1. C10 内参标定
2. Alicia-M eye-in-hand 手眼标定
3. tool0→spray_nozzle_link 喷嘴外参实测
4. 1 m 平面干式对准
5. 湿喷落点测试并修正 pixel_trim
6. 单叶闭环
7. 四树连续闭环
```

三个产物彼此不能替代：

| 产物 | 解决的问题 | 默认路径 |
|---|---|---|
| C10 内参 | 像素射线、畸变 | `~/.ros/camera_info/c10.yaml` |
| 手眼外参 | `tool0→camera_color_optical_frame` | `~/.ros/wvcsc_calibration/c10_handeye.yaml` |
| 喷嘴外参 | `tool0→spray_nozzle_link`、工距、落点微调 | `~/.ros/wvcsc_calibration/nozzle.yaml` |

实机任务模式默认 `require_nozzle_calibration:=true`。CameraInfo、手眼或喷嘴标定缺失时，前置检查必须阻止自动喷洒。

## 4. 喷嘴精准偏移补偿

### 4.1 坐标系与标定文件

```text
tool0
  ├── camera_color_optical_frame
  └── spray_nozzle_link
```

`spray_nozzle_link +Z` 是喷流中心轴。仿真名义喷嘴与 `tool0` 重合；实机从外部文件加载：

```yaml
schema_version: 1
parent_frame: tool0
child_frame: spray_nozzle_link
translation: {x: 0.0, y: 0.0, z: 0.0}
rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
working_distance_m: 1.0
working_distance_tolerance_m: 0.05
pixel_trim: {u: 0.0, v: 0.0}
```

模板位于 `wvcsc_calibration/config/nozzle.example.yaml`。实测结果只写入用户目录，不提交设备私有标定值。

### 4.2 投影模型

视觉伺服不再把病叶强制对准图像中心，而是对准喷嘴轴线在固定工距平面上的投影。设相机坐标系中的喷嘴原点为 `o`，喷流方向为 `d`，固定平面深度为 `Z=1.00 m`：

```text
d = R_camera_nozzle × [0, 0, 1]
lambda = (Z - o.z) / d.z
p = o + lambda × d

u_nozzle = fx × p.x / p.z + cx
v_nozzle = fy × p.y / p.z + cy

desired_u = u_nozzle + pixel_trim.u
desired_v = v_nozzle + pixel_trim.v
```

`wvcsc_visual_servo/aim_compensation.py` 负责几何校验和投影。以下任一条件成立即拒绝 Align Goal 或喷洒：

- CameraInfo、TF 或标定无效；
- 喷嘴轴 `d.z ≤ 0.2`；
- 射线与 1 m 平面的交点在喷嘴后方；
- 补偿像素超出安全图像区域；
- 观察候选工距不在 0.95–1.05 m；
- 对准后目标、锁状态或补偿状态复核失败。

当前名义安装在 1 m 处约对应图像中心 `u+0 px、v+28 px`。运行时必须使用真实 CameraInfo 与 `camera→spray_nozzle_link` TF 动态计算，不能硬编码 28 px。

### 4.3 对准与喷洒门控

```text
目标重心到工作区
→ 计算喷嘴投影点
→ 视觉伺服误差改为 target - nozzle_projection
→ 双轴误差 ≤1.5 px 并连续稳定 0.5 s
→ 再次检查工距与安全状态
→ /spray/execute
```

终端关键日志：

```text
[AIM] source=fixed range=1.000m nozzle_frame=spray_nozzle_link
[AIM] target_pixel=(640.0,388.1) trim=(0.0,0.0)
[AIM] aligned error_px=(1.1,-0.7) estimated_plane_error_mm=2.6
```

`estimated_plane_error_mm` 是固定深度平面上的理论图像误差，不包含手眼误差、喷嘴安装误差、叶片摆动和喷流扩散。

### 4.4 固定工距与泵电流

第一版没有可信深度输入，观察距离固定为 1.00 m：

```yaml
observation_distance_min_m: 1.00
observation_distance_max_m: 1.00
observation_distance_step_m: 0.10
```

泵电流只用于标定射程、覆盖半径和流量，不能代替横向几何补偿。湿喷后只允许用 `pixel_trim` 修正稳定、可重复的系统落点偏差。后续如增加 ToF，应新增 `aim_range_source`，不得把未经标定的深度直接接入喷洒。

## 5. Alicia-M 自适应自动手眼标定

### 5.1 为什么不能照搬 Fairino 姿态

迁移只复用 easy_handeye2 服务、ArUco 质量门控、多算法求解、离群剔除和报告流程。Fairino 的绝对关节角、双 MoveGroup、专用 IK、笛卡尔服务、规划器名和工作空间全部不迁移。

Alicia-M 候选必须基于当前真实标记位置和本机运动学自适应生成，再逐个通过：

```text
碰撞 IK
→ 六关节完整性
→ URDF Jacobian 条件数 <14
→ 最小关节余量 ≥0.22 rad
→ OMPL RRTConnectFast 预规划
→ 执行并停稳
→ 连续 ArUco 图像质量门控
→ easy_handeye2 take_sample
```

候选共 21 个，包括当前姿态、roll `±8°/±14°`、径向 `±30/±45 mm`、水平和垂直 `±6°/±10°`、四个组合视角附加 `±5°` roll。少于 14 个安全候选时整轮失败，不放宽安全阈值。

### 5.2 采样、重心和求解门控

- 有效样本：最少 15，目标 18，最多 22；
- 求解子集：离群剔除后不得少于 14；
- 每姿态稳定 10 帧，停稳等待 1 s；
- 标记距离 0.25–0.80 m，图像边缘余量至少 60 px；
- 中心标准差不超过 4 px，深度标准差不超过 3 mm，角度标准差不超过 0.8°；
- 相邻样本平移差至少 6 mm或旋转差至少 3°；
- 总平移跨度至少 40 mm、总旋转跨度至少 20°；
- 中心偏差大于 45 px 时最多重心三次，单次不超过 3 mm，累计不超过 10 mm；
- Park、Horaud、Tsai-Lenz 三算法平移差不超过 10 mm、旋转差不超过 2°；
- 静态标记位置 RMS 不超过 5 mm、姿态 RMS 不超过 1°；
- `tool0→camera` 平移范数不超过 0.30 m。

每次按 `s` 开始前会清空 easy_handeye2 服务端旧样本。质量不合格时按固定标记残差逐个删除最差样本并重新执行三算法求解，最多剔除到 14 个；仍不达标则失败，不保存结果。成功后使用临时文件、`fsync` 和原子替换导出。

### 5.3 三终端操作

终端一启动 Alicia-M、MoveIt、C10、ArUco 和 easy_handeye2 服务：

```bash
ros2 launch wvcsc_calibration c10_handeye.launch.py
```

终端二启动自动采集器：

```bash
ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file $(ros2 pkg prefix wvcsc_calibration)/share/\
wvcsc_calibration/config/auto_handeye_alicia.yaml
```

在该终端输入并回车：

```text
s 或空 Enter  启动一轮全新采集
q             取消并返回本轮起始关节姿态
Ctrl+C        取消并退出
```

终端三提供实时安全介入：

```bash
ros2 run wvcsc_arm_task motion_control_keyboard
```

```text
SPACE  stop并锁定
h      reset并执行HOME
r      HOME_LOCKED后解除锁定
x      立即stop；完整实机系统同时请求/safety/controlled_abort
```

独立手眼标定不启动底盘安全节点，因此此时 `x` 至少保证机械臂立即 stop；
完整作业系统中才继续执行底盘零速、停稳和 HOME 的受控中止链。

任何 stop、reset、HOME_LOCKED 或物理急停都会立即作废当前标定会话并取消机械臂 Goal。完成 `h → HOME_LOCKED → r` 后，必须回到终端二重新按 `s`；不允许从中断样本继续求解。

## 6. 受控中止与恢复

Nav2 输出固定经过安全门控：

```text
Nav2 /cmd_vel_nav → wvcsc_safety → /cmd_vel → wtb_car_driver
```

`/safety/controlled_abort` 顺序固定为：

```text
取消 mission/Nav2
→ 立即并持续发布底盘零速度
→ 发布机械臂 stop
→ 等待底盘连续停稳 1 s
→ 确认物理急停已解除
→ reset并执行HOME
→ HOME_LOCKED
→ 人工resume后才允许重新工作
```

物理急停激活期间只允许零速度和 stop，不允许自动 HOME。

## 7. 构建、测试与现场验收

2026-07-21 的提交前代码验收结果：14 个相关 ROS 包构建成功，`colcon test`
汇总为 `261 tests / 0 errors / 0 failures / 0 skipped`；实机和标定 launch
参数均可解析。当前保留的两份仿真权重可由 Ultralytics 直接加载：

```text
yolov8s_sim.pt      task=detect  names={0: tree}
yolov8s_seg_sim.pt  task=segment names={0: healthy_fruit, 1: diseased_fruit}
```

这些结果只证明代码、接口和仿真模型契约，不代表真实喷嘴落点已经验收。

代码侧：

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

现场验收不得省略：

1. 内参重投影误差小于 0.5 px；
2. 两次独立手眼标定差异不超过 5 mm / 1°；
3. 1 m 平面干式对准 P95 不超过 10 mm；
4. 湿喷落点 P95 不超过 15 mm；
5. 工距超限、目标丢失、TF失效或安全锁定时喷洒 Goal 数必须为 0；
6. stop 后无残留 MoveIt/Nav2/Spray Goal；
7. 每棵树三次重新驶离再导航，停靠误差均不超过 0.12 m / 0.12 rad；
8. 连续三轮四树任务：导航 12/12、错误树关联 0、视觉伺服 100%、喷洒 100%、`skipped_targets=0`。

若某棵树三次直达停靠有两次失败，不允许放宽公差，应将该树升级为预停靠点加精停靠的两段式方案。

## 8. 当前未完成项

- 用户提供并验证实机 `yolov8s_real.pt` 与 `yolov8s_seg_real.pt`；
- 在固定支架最终安装后生成真实三份标定文件；
- 获取 Alicia-M 厂家真实速度、加速度、停止距离与负载限制；
- 标定 `base_footprint` 物理测量基准、LiDAR/IMU外参和喷嘴湿喷落点；
- 在真实玉米叶片 0.8–1.6 m 高度范围完成单叶及四树验收；
- 真实泵阀接入前继续使用 5 s 模拟 `/spray/execute`。

在以上现场项完成前，状态应记录为“代码闭环完成，真实喷洒验收未完成”。

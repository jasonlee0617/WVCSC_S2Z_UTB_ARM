# WVCSC ARMSpray 视觉喷洒闭环实施方案

> **更新日期**：2026-07-20
> **当前阶段**：仿真闭环已完成 `4/4`，所有 detected diseased_fruit 100% 喷洒成功。对准中位 `1.4s`、最终单轴误差 ≤ `2px`。控制链：30 Hz IBVS → 100 Hz MoveIt Servo。
> **验证结果**：24 个包全部构建通过，`196 tests, 0 errors, 0 failures, 0 skipped`。
> **下一步**：① C10 相机内参/手眼标定 → ② 真实 Alicia-M 接入 → ③ 统一实机 launch 与仿真→实机兼容迁移。

---

## 0. 工作区现状（2026-07-20）

### 0.1 包清单（24 个）

| 层 | 包 | 类型 | 说明 |
|----|-----|------|------|
| 硬件驱动 | `serial`, `can_bridge`, `fdilink_ahrs_ROS2`, `wtb_car_driver`, `lslidar_driver`, `lslidar_msgs` | C++ | 底盘/CAN/IMU/LiDAR |
| 机械臂 | `alicia_m_descriptions`, `alicia_m_driver`, `alicia_m_moveit_config`, `alicia_m_bringup`, `alicia_m_calibration` | C++ | URDF/ros2_control/MoveIt/手眼标定 |
| SLAM/导航 | `my_cartographer`, `my_navigation2` | C++ | Cartographer SLAM + Nav2 |
| 运动控制 | `pymoveit2`, `trajectory_retime_server` | C++ | Python MoveIt2 封装 + 轨迹重定时 |
| 复合模型 | `wvcsc_description` | C++ | 统一 XACRO（底盘+Alicia+C10+喷嘴）|
| 仿真 | `wvcsc_simulation` | C++ | Gazebo 世界生成、AckermannSim、orchard assets |
| 接口 | `wvcsc_interfaces` | C++ | 自定义 ROS2 action/srv/msg |
| 任务编排 | `wvcsc_mission_manager`, `wvcsc_uav_gateway` | Python | 使命管理器 + Mock/Replay UAV |
| 感知 | `wvcsc_rgb_vision`, `wvcsc_c10_camera` | Python | YOLOv8n 两级推理 + C10 驱动 |
| 执行 | `wvcsc_arm_task`, `wvcsc_visual_servo` | Python | 喷洒状态机 + IBVS 伺服 |

### 0.2 已完成的收敛工作

- **数据采集解耦**：`capture_yolo_*.py`、`orchard_assets.py`、`yolo_seed_dataset.py` 迁入 `wvcsc_simulation/data_acquisition/`
- **Mock 模式移除**：`perception_mode` 参数和 `mock_vision` 节点已删除
- **控制链分层**：C10/YOLO(30Hz) → IBVS Twist(30Hz) → MoveIt Servo(100Hz) → ros2_control(100Hz)
- **观察位姿**：`ObservationOptimizer` 按距离/高度/方位角生成候选网格，经碰撞 IK、条件数(≤16.5)、关节余量(≥0.22rad) 筛选
- **目标重心**：`TargetRecenter` 在 IBVS 前执行 MoveIt 笛卡尔修正（触发阈值 48px，最大旋转 20°，迭代 2 次）
- **稳定校验**：重心修正后需 0.2s 内漂移 ≤ 4px 才允许喷洒

---

## 1. 仿真闭环状态

### 1.1 已完成

```text
四树坐标任务 (Mock UAV)
  → Nav2 导航到道路停靠位姿 (docking_lateral_offset=0.2m)
  → /odom 停稳确认 (1.0s, 线速度≤0.03m/s, 角速度≤0.03rad/s)
  → ObservationOptimizer 动态观察位姿 (条件数≤16.5, 关节余量≥0.22rad)
  → YOLOv8n Tree Detect (conf=0.10) → SCANNING_TREE
  → YOLOv8n-seg Fruit Seg (conf=0.20) → diseased_fruit 候选
  → QUEUING 去重排序 → 逐病果 TargetRecenter + IBVS
  → ALIGNING (30Hz, PID kp=4.0, fine_tolerance=1.5px, stable=0.5s)
  → SPRAYING (Spray Action)
  → 复检 RETURNING_TO_OBSERVE → 队列空 → HOME → MISSION_COMPLETED
```

### 1.2 关键性能指标

| 指标 | 当前值 |
|------|--------|
| IBVS 控制频率 | **30 Hz** |
| MoveIt Servo 周期 | **0.01s (100 Hz)** |
| 最终对准精度 | **≤ 2 px/轴** |
| 对准中位时间 | **1.4 s** |
| 对准最大时间 | **2.0 s** |
| 稳定判定 | 0.5s 内误差 ≤ 1.5px |
| 重心修正触发 | 像素偏差 > 48px |
| 重心最大旋转 | 20°/次, 迭代 ≤ 2 |

### 1.3 仿真启动命令

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
./run_system_sim.sh
# 等价于:
ros2 launch wvcsc_simulation system_sim.launch.py \
  use_nav2:=true auto_start_mission:=true \
  use_mock_uav:=true use_mission_manager:=true
```

---

## 2. 实机部署方案

### 2.1 第一步：C10 相机内参标定

**当前状态**：`c10_reference_calibration.yaml` 是占位参数（fx=fy=507.87, 零畸变）。

**标定步骤**：

1. 准备标定板：9×6 棋盘格，方格 20mm，打印贴在平板上
2. 启动相机：
   ```bash
   ros2 launch wvcsc_c10_camera c10_camera.launch.py
   ```
3. 用 `camera_calibration` 工具采集：
   ```bash
   ros2 run camera_calibration cameracalibrator \
     --size 8x5 --square 0.020 \
     --ros-args -r image:=/camera/color/image_raw
   ```
   在不同角度/距离采集 ≥ 30 张有效图像，直到 CALIBRATE 按钮亮起
4. 点击 CALIBRATE → 等待计算完成 → 点击 COMMIT 保存
5. 将生成的 `ost.yaml` 替换 `c10_reference_calibration.yaml`
6. **验证**：标定后重投影误差必须 < 0.5px；用 `ros2 topic echo /camera/color/camera_info` 确认 fx/fy/cx/cy 和畸变系数已更新

**涉及文件**：
- [c10_reference_calibration.yaml](src/wvcsc_c10_camera/config/c10_reference_calibration.yaml) → 替换
- [c10_usb_cam.yaml](src/wvcsc_c10_camera/config/c10_usb_cam.yaml) 中的 `camera_info_url` → 确认指向新文件

### 2.2 第二步：手眼标定（Eye-in-Hand）

**当前状态**：标定包已就绪（`alicia_m_calibration`），代码已测试，无真实标定结果。

**标定步骤**：

1. 在机械臂工作空间内固定 ArUco 码（建议 `DICT_4X4_50`, ID=0, 5cm）
2. 启动机械臂硬件 + MoveIt：
   ```bash
   ros2 launch alicia_m_bringup moveit_hardware.launch.py
   ```
3. 启动相机：
   ```bash
   ros2 launch wvcsc_c10_camera c10_camera.launch.py
   ```
4. 执行标定：
   ```bash
   ros2 launch alicia_m_calibration hand_eye_calibration.launch.py
   ```
   机械臂自动移动到 20 个标定位姿，每姿态采集 ArUco 检测
5. 标定结果保存到 `hand_eye_calibration_result.yaml`
6. **验证**：用 `verify_calibration.launch.py` 目视验证 TF 链 `tool0 → camera_link → aruco_marker_frame`

**涉及文件**：
- [hand_eye_calibration.launch.py](src/Alicia-M-ROS2/alicia_m_calibration/launch/hand_eye_calibration.launch.py)
- [hand_eye_calibration_result.yaml](src/Alicia-M-ROS2/alicia_m_calibration/config/hand_eye_calibration_result.yaml)
- [wvcsc_utb_alicia.urdf.xacro](src/wvcsc_description/urdf/wvcsc_utb_alicia.urdf.xacro) 中的 `c10_mount_xyz` / `c10_mount_rpy` → 用标定结果更新

### 2.3 第三步：统一实机 Launch 文件

**当前问题**：实机有三个独立启动入口，没有统一的顶层 launcher：
- `start_wtb_car_fdimu.launch.py` — 底盘+传感器
- `wtb_navigation2_fdimu.launch.py` — 底盘+导航
- `moveit_hardware.launch.py` — 机械臂

**方案**：新建 `wvcsc_simulation/launch/system_real.launch.py`，结构与 `system_sim.launch.py` 对齐，但将 Gazebo 仿真组件替换为真实硬件驱动。

```text
system_real.launch.py
├── [硬件] can_bridge (CAN → ROS2)
├── [硬件] fdilink_ahrs (IMU, 串口)
├── [硬件] lslidar_driver (LiDAR)
├── [硬件] wtb_car (底盘 CAN 驱动)
├── [硬件] C10 camera (usb_cam + watchdog)
├── [模型] robot_state_publisher (XACRO URDF)
├── [模型] joint_state_publisher
├── [控制] robot_localization EKF (odom + IMU 融合)
├── [导航] Nav2 (navigation_launch.py)
├── [机械臂] alicia_m_hardware + ros2_control
├── [机械臂] move_group (MoveIt2)
├── [机械臂] trajectory_retime_server
├── [机械臂] motion_control
├── [机械臂] spray_task
├── [视觉] two_stage_yolo (YOLOv8n)
├── [视觉] visual_servo (IBVS)
├── [视觉] moveit_servo
├── [任务] mission_manager
├── [任务] mock_uav_gateway / replay_uav_gateway
└── [可选] rviz2
```

**关键差异**（仿真 vs 实机）：

| 项目 | 仿真 (`system_sim.launch.py`) | 实机 (`system_real.launch.py`) |
|------|------|------|
| `use_sim_time` | `true` | **`false`** |
| 机器人模型 | Gazebo spawn_entity | **仅 robot_state_publisher** |
| 底盘运动 | `ackermann_sim.py` (Twist→odom) | **CAN bridge + wtb_car (真实底盘)** |
| LiDAR | `pointcloud_to_laserscan` (仿真) | **真实 lslidar_driver** |
| IMU | 无（仿真 EKF 用 odom） | **fdilink_ahrs** |
| 相机 | Gazebo 插件 | **usb_cam + 真实 C10** |
| 机械臂 ros2_control | `gazebo_ros2_control/GazeboSystem` | **`alicia_m_driver/AliciaHardwareInterface`** |
| 机械臂 base_frame | `alicia_base_link` | **`base_link`**（通过 `camera_look_at_pose` 的 `tree_hint` 自动适配） |
| 速度缩放 | `velocity_scaling=0.40` | **`velocity_scaling=0.10`**（安全保守） |
| `check_collisions` | `false`（仿真） | **`true`**（实机必须） |

### 2.4 第四步：代码兼容迁移

仿真与实机的主要代码是**同一套**——`spray_task.py`、`visual_servo_node.py`、`two_stage_yolo.py` 等。差异仅在配置。

**需要为实机准备的配置文件**：

| 文件 | 修改内容 |
|------|---------|
| `arm_task.yaml` (实机版) | `base_frame: base_link`（非 `alicia_base_link`），降低 `max_linear_speed` 和 `max_angular_speed` |
| `visual_servo.yaml` (实机版) | 更新 `min_confidence`、PID 增益建议降低至 kp=1.0~2.0 |
| `moveit_servo.yaml` (实机版) | `check_collisions: true`，`use_gazebo: false`，`publish_period: 0.01` (100Hz) |
| `mission_manager.yaml` (实机版) | `home_x/home_y/home_yaw` 修正为真实 HOME 位姿 |
| `mock_targets.yaml` | 目标坐标改为真实果园的 GPS/Map 坐标 |

**建议**：在 `config/` 下创建 `real/` 子目录存放实机配置，launch 文件通过 `config_mode:=real` 参数切换。

### 2.5 第五步：实机调试顺序

```text
□ 底盘 CAN 通信验证
  → ros2 topic echo /car_odom (确认轮式里程计正常)
  → ros2 topic echo /cmd_vel (确认速度指令能被底盘执行)

□ IMU 数据验证
  → ros2 topic echo /imu (确认角速度/加速度数据正常)

□ LiDAR 数据验证
  → ros2 topic echo /scan (确认激光数据正常, range 有效)

□ EKF 融合验证
  → ros2 topic echo /ekf_odom (确认融合里程计正常)
  → ros2 run tf2_tools view_frames (确认 map→odom→base_footprint TF 树完整)

□ C10 相机验证
  → ros2 topic hz /camera/color/image_raw (确认 30Hz 稳定)
  → rviz2 查看 Image 显示 (确认曝光/白平衡正常)

□ 机械臂 ros2_control 验证
  → ros2 topic echo /joint_states (确认 7 轴关节数据)
  → ros2 action list | grep arm_controller (确认 FollowJointTrajectory Action 可用)

□ 机械臂运动验证 (低速)
  → 从 HOME 到简单位姿的 MoveIt 规划测试
  → 验证碰撞检测、关节限位保护

□ YOLO 推理验证
  → 采集一帧真实 C10 图像 → 用部署的权重离线推理 → 确认 tree/fruit 检测正常

□ 单树手动闭环
  → 手动发送 ExecuteSpray Goal → 验证完整七阶段流程

□ 四树自动闭环
  → Mock UAV + mission_manager 真实果园场景
```

### 2.6 关键安全护栏（实机必须）

| 措施 | 文件 | 参数 |
|------|------|------|
| 关节速度限制 | URDF | joint velocity = 10 rad/s → scaling 0.1 = **1.0 rad/s** |
| 关节加速度限制 | `joint_limits.yaml` | 必须从厂家获取真实值，替换当前默认 1 rad/s² |
| Servo 碰撞检测 | `moveit_servo.yaml` | `check_collisions: true` |
| Servo 奇异点 | `moveit_servo.yaml` | `hard_stop_singularity_threshold: 30.0` |
| 关节限位 | `moveit_servo.yaml` | `joint_limit_margin: 0.10` |
| 运动锁 | `motion_control` | 急停按钮 → 发布 `stop` → 永久锁定 |
| 喷洒互锁 | spray_task | 未对准/目标丢失/Servo异常 → 禁止开阀 |
| 人员安全区 | 物理 | 机械臂工作半径内设置物理隔离 |

---

## 3. 接口边界（仿真与实机统一）

| 接口 | 职责 | 实机变更 |
|------|------|---------|
| `/uav/disease_trees` | 树级任务列表 | 无变化 |
| `/mission/plan` | 树坐标与停靠位姿 | 无变化 |
| `/navigate_to_pose` | Nav2 导航 | 无变化 |
| `/arm/execute_spray` | 一棵树的完整作业 | 无变化 |
| `/vision/align_target` | 单病果 XY 对准 | 无变化 |
| `/spray/execute` | 喷洒执行 | 仿真模拟 → **真实泵阀** |
| `/camera/color/image_raw` | RGB 图像 | Gazebo 插件 → **C10 usb_cam** |
| `/camera/color/camera_info` | 内参 | 占位参数 → **真实标定结果** |
| `/joint_states` | 关节角 | Gazebo 仿真 → **真实编码器** |
| TF `tool0→camera_link` | 手眼外参 | 临时占位 → **标定结果** |

---

## 4. 构建与测试

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash

# 全量构建
colcon build --symlink-install
source install/setup.bash

# 运行全部测试
colcon test --event-handlers console_direct+
colcon test-result --all
```

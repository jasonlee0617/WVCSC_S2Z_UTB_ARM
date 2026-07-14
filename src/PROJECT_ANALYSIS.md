# WVCSC_S2Z_UTB_ARM 项目分析文档

## 项目概述

本项目是一个基于 **ROS2 Humble** 的无人车+机械臂复合机器人系统，目标平台为 **ARM aarch64**（兼容 x86_64），部署在一台 **阿克曼转向底盘**（猛犸IV / 极速IB系列）上，并搭载 **Alicia-M 六轴机械臂**（Synria Robotics 制造）。软件栈覆盖从底层硬件驱动、传感器融合、SLAM 建图、自主导航、机械臂运动规划与控制的完整功能链路，并提供 Gazebo 仿真环境。

本工作区 **共含 15+ 个 ROS2 包**，分为三大类：
- **无人车底层**：串口/CAN驱动、IMU/GPS驱动、激光雷达驱动、底盘驱动、SLAM、导航
- **机械臂**：URDF模型、硬件驱动(ros2_control)、MoveIt2运动规划、手眼标定、抓取模块
- **仿真与集成**：Gazebo仿真、复合机器人模型、喷雾任务、Nav2导航仿真

---

## 当前阶段状态与未来 48 小时目标（2026-07-14）

本节是当前开发基线，优先级高于后文中尚未随代码更新的历史描述。评估依据是本工作区源代码、启动文件、单元测试和已经完成的 Gazebo/MoveIt 仿真检查；百分比是工程进度估算，不是性能指标。

技术依据为项目文档《智能农林病虫害空地协同自主防治系统技术方案_V2.0_完整详细版》（2026 年 7 月），重点采用其中“无人机粗引导→小车 Nav2→机械臂近场执行、Mock/Replay/Live 统一入口、仿真先行”的路线。

### 本次目标

在 Gazebo 的统一小车+Alicia-M 模型中完成以下最小闭环：

```text
Mock 无人机回传病树目标坐标
        ↓
任务编排器校验坐标/TF并生成小车停靠位姿
        ↓
Nav2 NavigateToPose 自动导航并确认小车停稳
        ↓
MoveIt2 规划 Alicia-M 观察/喷洒/收回动作
        ↓
记录成功或失败，失败时停止后续目标
```

这里按技术方案 V2.0 的“Mock 无人机”路线执行：48 小时内模拟的是无人机完成巡检后回传的病树任务坐标，不实现真实飞控、相机识别或农业遥感算法。坐标默认固化到 `map`；如果“无人机回传坐标系信息”实际指无人机机体位姿而不是病树目标坐标，应在下一轮对话中确认，接口会相应增加 `uav_base_link` TF。

### 已完成和已验证的能力

| 子系统 | 当前状态 | 证据/边界 |
|---|---|---|
| 统一机器人模型 | 已完成基础合并 | `wvcsc_description/urdf/wvcsc_utb_alicia.urdf.xacro` 组合 `wtb_car.xacro` 与 Alicia-M 模型，机械臂根为 `alicia_base_link`。 |
| Gazebo 启动编排 | 已完成可运行基线 | `wvcsc_simulation/launch/system_sim.launch.py` 可编排 Gazebo、robot_state_publisher、控制器、MoveIt、重定时服务、运动控制和喷洒节点；当前 `use_nav2`、`use_rviz` 默认值为 `true`。 |
| 小车运动仿真 | 已完成基础能力 | `AckermannSim` 订阅 `/cmd_vel`，发布 `/odom`、`/ekf_odom` 和 `odom→base_footprint`，并带指令超时停车。 |
| Alicia-M MoveIt 仿真 | 已完成基础闭环 | 已验证关节、位姿和非奇异姿态下笛卡尔规划执行；控制器为 `joint_state_broadcaster`、`arm_controller`、`gripper_controller`。 |
| 轨迹安全链路 | 已完成 | 官方 `pymoveit2` 4.2.0 保持未修改；笛卡尔轨迹执行为 `plan → /retime_trajectory → 校验 → execute`，重定时失败或轨迹非法时禁止执行。 |
| stop/reset/resume | 已完成仿真验证 | `reset` 目标为 `HOME=[0.0,0.0,0.0,0.0,0.0,0.0]`，先停止、再张开夹爪、再回 HOME；失败保持锁定。 |
| 喷洒动作 | 已完成“模拟动作” | `/arm/execute_spray` 已能执行观察姿态→等待喷洒时长→HOME；目前仍是 `Trigger` 服务并异步返回，尚不适合作为多目标任务编排的最终完成信号。 |
| 测试与构建 | 已完成源代码级验证 | `wvcsc_arm_task` 15 个 Python 测试通过，`trajectory_retime_server` 2 个 C++ 测试通过；曾完成 9 包隔离构建。正式工作区仍需按下方命令重新构建并验证安装空间。 |

### 当前完成度估算

| 能力域 | 完成度 | 尚缺的关键内容 |
|---|---:|---|
| 小车实车底层（CAN/IMU/LiDAR/里程计） | 70% | 统一任务接口、整车新 footprint 和比赛场地回归。 |
| Alicia-M 模型、MoveIt 与仿真控制 | 75% | `tool0` 碰撞几何、动作完成事件、真实关节加速度上限。 |
| 统一 Gazebo 复合模型 | 70% | 与 orchard 地图坐标、导航代价地图和停靠位姿的联合标定。 |
| Nav2 在统一仿真中的自动导航 | 40% | 目标点 Action 端到端成功、停稳互锁、多目标和异常恢复。 |
| 无人机 Mock 坐标回传 | 0% | `wvcsc_uav_gateway`、Mock 数据文件、坐标校验和任务列表接口。 |
| 任务管理与空地协同 | 10% | `wvcsc_mission_manager` 状态机，串联 Nav2 与机械臂。 |
| 视觉、真实喷洒和 Web | 0%～10% | C10 RGB、手眼/单目对准、喷洒 IO、网页后端尚未接入。 |
| 实机 Alicia 接入 | 25% | 机械臂到货、串口/控制器连续运行、低速和安全验收。 |

按“比赛可完整运行闭环”衡量，当前约 **40%～45%**；按底层软件储备衡量高于该数值。未来 48 小时的验收目标不是完成全部项目，而是把“无人机 Mock → Nav2 单/多目标 → Alicia-M 模拟喷洒”的仿真闭环提升到可重复运行。

### 仍待完成的核心工作

1. 冻结 `map`、`odom`、`base_footprint`、`base_link`、`alicia_base_link`、`tool0` 的 TF 责任，确保无人机目标在检测时转换并固化为 `map` 坐标。
2. 新增 Mock 无人机入口（建议 `wvcsc_uav_gateway`），从 YAML/JSON 读取目标列表，发布文档定义的 `/uav/disease_trees`；Mock、Replay、Live 后续都走同一接口。
3. 新增任务编排入口（建议 `wvcsc_mission_manager`），逐目标调用 `nav2_msgs/action/NavigateToPose`，导航成功且底盘速度稳定后才能触发机械臂。
4. 为 `/arm/execute_spray` 增加可等待的完成语义（优先迁移为 `ExecuteSpray.action`；48 小时内也可先增加明确的状态/结果话题），避免任务管理器把“请求已接受”误判为“喷洒完成”。
5. 完成导航失败、机械臂失败、`stop/reset` 和急停后的状态迁移；任何失败都不得自动发送下一目标。
6. 修正 `tool0` 无碰撞几何和 Alicia-M 关节加速度参数。实机启动前必须取得厂家限制，仿真与硬件继续使用同一 URDF/MoveIt 语义配置。

### 今天（Day 1）工作安排：先打通无人机 Mock → 小车导航

| 顺序 | 实现内容 | 验收输出 |
|---:|---|---|
| 1 | 冻结接口和坐标约定：目标消息含 `mission_id`、`tree_id`、时间戳、`frame_id`、`confidence`、`map_position` 和停靠位姿。 | 一页接口说明；明确所有目标最终使用 `map`。 |
| 2 | 创建 `wvcsc_uav_gateway` 的 Mock 节点和 `config/mock_targets.yaml`；发布 2 个固定 orchard 目标，支持重复发布/幂等任务号。 | `ros2 topic echo /uav/disease_trees` 能看到完整目标列表。 |
| 3 | 创建任务编排器最小骨架，只实现“接收一个目标→生成/读取停靠位姿→调用 `NavigateToPose`”。 | 命令行或测试节点可发送目标，不把坐标写死在 Nav2 节点中。 |
| 4 | 用 `system_sim.launch.py use_nav2:=true` 运行 Gazebo+Nav2，检查 `map→odom→base_footprint→base_link` TF、`/cmd_vel` 和 `/odom`。 | 小车到达单个目标，Nav2 Action 返回 `SUCCEEDED`，到点后速度回到零。 |
| 5 | 保存启动日志和验收命令，记录地图原点、目标点、到点误差和失败原因。 | 一次可复现的 Day 1 运行记录。 |

### 明天（Day 2）工作安排：导航成功 → 机械臂喷洒 → 多目标

| 顺序 | 实现内容 | 验收输出 |
|---:|---|---|
| 1 | 在任务编排器中加入 `NAVIGATING → ARRIVED → ARM_SPRAYING → COMPLETED` 状态；只有导航成功、停稳持续约 1～2 秒才允许喷洒。 | 状态转换日志完整且顺序不可跳跃。 |
| 2 | 接入 `wvcsc_arm_task`：观察姿态、模拟喷洒等待、HOME；补充可等待的喷洒完成/失败结果。 | 一个目标完成后，机械臂回 HOME，编排器收到明确结果。 |
| 3 | 实现两个目标的顺序执行、最大重试/跳过策略和返回待命点；任何 `stop/reset` 后锁定期间不得接收新目标。 | 连续完成 2 个目标；导航或机械臂失败时不误触发下一个目标。 |
| 4 | 在 RViz/Gazebo 中显示目标、路径、车体和机械臂 TF；保留 headless 启动方式用于回归。 | 图形和终端均能追踪当前目标及状态。 |
| 5 | 运行完整验收：Mock → Nav2 → 喷洒 → HOME，重复至少 3 次并记录结果。 | 形成“已通过/失败原因/下一步”表格，作为后续网页端对话基线。 |

### 48 小时验收标准与明确不纳入范围

- 统一仿真启动后无 TF 断树；无人机目标的 `frame_id`、时间戳和坐标范围校验有效。
- Nav2 至少完成一个固定目标的 `NavigateToPose`，小车到达后停稳；随后才允许机械臂动作。
- 机械臂动作只经 MoveIt2/ros2_control；重定时失败不执行原始轨迹；`stop/reset/resume` 仍满足锁定语义。
- 喷洒成功必须有可观察的完成结果；导航失败或喷洒失败不得自动执行下一目标。
- 本轮不承诺真实无人机飞控/视觉识别、C10 视觉对准、真实水泵/阀门、网页 UI、实机 Alicia 加速度标定或多无人机调度。

### 需要在网页端继续确认的 4 个决策

1. “无人机回传坐标系信息”是否按 V2.0 解释为病树目标坐标列表；默认使用 `map`，而不是只发布无人机自身位姿。
2. 今明两天先做 2 个固定目标顺序执行，还是只验收 1 个目标后再扩展多目标；默认先单目标、再双目标。
3. 喷洒接口是否接受从当前 `Trigger` 过渡到 `ExecuteSpray.action`；默认采用可等待 Action/结果话题。
4. 验收是否以 Gazebo GUI+RViz 为主、headless 为回归；默认两者都保留，自动化检查使用 headless。

### 当前基线构建命令

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src src/Alicia-M-ROS2 \
  --packages-select pymoveit2 trajectory_retime_server wvcsc_arm_task \
  alicia_m_descriptions alicia_m_driver alicia_m_moveit_config \
  alicia_m_bringup wvcsc_description wvcsc_simulation
source install/setup.bash
ros2 launch wvcsc_simulation system_sim.launch.py
```

回归时可先使用 `use_nav2:=false use_rviz:=false gui:=false` 验证机械臂和 Gazebo 基线，再打开 Nav2 验收空地闭环。当前仍需在正式工作区重新生成 `install/` 后确认完整构建，不把临时隔离构建目录当成项目安装空间。

---

## 技术栈一览

| 层面 | 使用的技术/框架 |
|------|----------------|
| **中间件** | ROS2 Humble (ament_cmake, rclcpp, rclpy) |
| **构建系统** | CMake + ament + colcon |
| **编程语言** | C++14/17（驱动/算法层），Python 3（启动/导航/机械臂控制/仿真），Lua（Cartographer 配置） |
| **数学库** | Eigen3（四元数、姿态坐标变换） |
| **SLAM** | Google Cartographer ROS2 |
| **导航** | Nav2 (Navigation2) |
| **状态估计** | robot_localization (EKF 扩展卡尔曼滤波) |
| **机械臂规划** | MoveIt2 + OMPL + Pilz Industrial Motion Planner |
| **机械臂控制** | ros2_control (JointTrajectoryController, GripperActionController) |
| **仿真** | Gazebo Classic (SDF 1.6) |
| **传感器驱动** | FDILink AHRS/IMU/GPS，镭神 LiDAR，轮式编码器，Intel RealSense D405 |
| **硬件通信** | CAN 总线 (libcontrolcan)，串口 (serial 库)，MQTT (mosquittopp)，UART (Synria 协议) |
| **可视化** | RViz2，PyQt5 自定义 GUI |
| **机器人模型** | URDF/Xacro |
| **目标平台** | ARM aarch64 / x86_64，Linux (Ubuntu) |

---

# 第一部分：无人车底层

## 1. serial — 跨平台串口库

- **类型**：纯 C++ 库（非 ROS 节点）
- **版本**：1.2.1，MIT 许可证
- **上游**：`wjwwood/serial`
- **功能**：提供 POSIX/Windows 跨平台串口通信 API，包括设备打开、读写、参数配置（波特率、校验位、停止位、流控、超时）。被 `fdilink_ahrs` 和 `alicia_m_driver` 包依赖使用。
- **主要文件**：[src/serial.cc](src/serial/src/serial.cc)，[examples/serial_example.cc](src/serial/examples/serial_example.cc)

---

## 2. can_bridge — CAN 总线桥接节点

- **框架**：ROS2 + rclcpp (C++)
- **依赖**：rclcpp, can_msgs, libcontrolcan.so
- **功能**：
  - 将 CAN 硬件数据与 ROS2 话题双向桥接
  - 管理 2 路 CAN 通道（CAN1 / CAN2）
  - CAN → ROS2：独立线程 50ms 周期轮询，封装为 `can_msgs::msg::Frame` 发布到 `/can_tx_1` `/can_tx_2`
  - ROS2 → CAN：订阅 `/can_rx_1` `/can_rx_2`，转换为 CAN 帧写入硬件
  - 500kbps 波特率，支持设备断线自动重连
  - 根据 CPU 架构自动选择 x86_64 或 arm64 的动态库
- **主要文件**：[src/can_bridge_node.cpp](src/can_bridge/src/can_bridge_node.cpp)

---

## 3. fdilink_ahrs_ROS2 — FDILink AHRS/IMU/GPS 姿态传感器驱动

- **框架**：ROS2 + rclcpp (C++)，Eigen3 姿态运算
- **依赖**：rclcpp, sensor_msgs, geometry_msgs, nav_msgs, tf2, serial, Eigen3
- **通信方式**：串口（默认 `/dev/ttyUSB0`，921600bps）
- **协议**：自定义二进制帧协议

  帧结构：`帧头(0xFF)` + `数据类型` + `数据长度` + `序列号` + `CRC8(帧头校验)` + `CRC16(数据校验)` + `数据区` + `帧尾`

  支持的数据类型：

  | 类型 | 含义 | 数据长度 |
  |------|------|----------|
  | `TYPE_IMU` (0x40) | 陀螺仪/加速度计/磁力计/温压 | 56 字节 |
  | `TYPE_AHRS` (0x41) | 欧拉角 + 四元数 | 48 字节 |
  | `TYPE_INSGPS` (0x42) | 惯导融合位置/速度 | 72 字节 |
  | `TYPE_GEODETIC_POS` (0x43) | GPS 经纬度/海拔 | 32 字节 |

- **发布的 ROS 话题**：

  | 话题名 | 消息类型 | 内容 |
  |--------|----------|------|
  | `/imu` | `sensor_msgs/Imu` | 四元数姿态 + 角速度 + 线加速度 |
  | `/gps/fix` | `sensor_msgs/NavSatFix` | GPS 经纬度、海拔 |
  | `/euler_angles` | `geometry_msgs/Vector3` | 欧拉角 (Roll/Pitch/Yaw) |
  | `/mag_pose_2d` | `geometry_msgs/Pose2D` | 磁力计航向角 |
  | `/magnetic` | `geometry_msgs/Vector3` | 磁力计三轴原始数据 |
  | `/system_speed` | `geometry_msgs/Twist` | 本体坐标系速度 |
  | `/NED_odometry` | `nav_msgs/Odometry` | NED 坐标系位置+速度 |

- **关键特性**：
  - CRC8 帧头校验 + CRC16 数据校验（预计算 CRC 表）
  - 序列号连续性检测与掉帧统计
  - 支持坐标系变换——通过 Eigen 四元数乘法将传感器坐标系转为 ROS 标准坐标系
  - 含 `imu_tf_node` 用于广播 IMU 到基座的 TF 变换
- **主要文件**：[src/ahrs_driver.cpp](src/fdilink_ahrs_ROS2/src/ahrs_driver.cpp), [include/fdilink_data_struct.h](src/fdilink_ahrs_ROS2/include/fdilink_data_struct.h), [src/crc_table.cpp](src/fdilink_ahrs_ROS2/src/crc_table.cpp)

---

## 4. wtb_car_driver — 底盘驱动与轮式里程计

- **框架**：ROS2 + rclcpp (C++)，MultiThreadedExecutor
- **依赖**：rclcpp, geometry_msgs, nav_msgs, tf2_ros, can_msgs, sensor_msgs, mosquittopp (MQTT)
- **功能**：底盘控制核心，负责接收导航指令并转换为 CAN 控制指令，同时解析底盘 CAN 反馈生成轮式里程计。

### 自定义消息

```plaintext
# CarMsg.msg
std_msgs/Header header
float64 speed       # 车速 m/s
float64 angle       # 转向角 rad
char battery        # 电量百分比
```

### 阿克曼运动学模型

基于阿克曼转向模型进行轮式里程计推算：

```
omega = v * tan(delta) / wheelbase
dx = v * cos(theta) * dt
dy = v * sin(theta) * dt
```

### CAN 通信协议

| CAN ID | 方向 | 内容 |
|--------|------|------|
| `0x18C4D2D0` | 发送 | 控制指令：档位(4bit) + 目标速度(16bit, 0.001m/s/bit) + 目标转向角(16bit, 0.01°/bit) + 心跳计数(4bit) + BCC校验 |
| `0x18C4D2EF` | 接收 | 底盘反馈：档位 + 当前速度 + 当前转向角 |
| `0x18C4E2EF` | 接收 | 电池电量信息 |

### 两个节点

**`wtb_car`**（功能完整版）：
- 订阅 `/cmd_vel`（标准导航速度指令）和 `/twist_cmd`（Autoware 带时间戳速度指令）
- 订阅 `/run_static` 运行状态控制（"start"/"stop"）
- 发布 `/car_odom` 里程计和 `/wtb_car_message` 自定义信息
- 参数可校准：轮距、速度系数、转向零偏、最小速度、最大速度、停止时间阈值

**`wtb_car_only`**（精简版）：
- 不含运行状态控制和自定义消息发布

### MQTT 客户端

基于 mosquittopp 库，支持远程遥测或云端通信。

### EKF 传感器融合配置

提供多种 `robot_localization` EKF 配置：
- `ekf_wtb_fdimu.yaml` — 主配置：融合轮式里程计 + FDILink IMU
- `ekf_only_odom.yaml` — 仅轮式里程计
- `ekf_only_imu.yaml` — 仅 IMU
- `ekf_wtb_autoware.yaml` — Autoware 集成配置

### URDF 模型

[xacro 模型](src/wtb_car_driver/urdf/wtb_car.xacro) 描述阿克曼转向车辆：
- 车身：1.30m × 0.80m × 1.00m（长×宽×高）
- 4 轮（前轮转向 + 后轮驱动）
- 激光雷达安装位（前 0.36m 处）
- IMU 安装位（底盘中心下方）

**主要文件**：[src/wtb_car.cpp](src/wtb_car_driver/src/wtb_car.cpp), [src/wtb_car_only.cpp](src/wtb_car_driver/src/wtb_car_only.cpp), [urdf/wtb_car.xacro](src/wtb_car_driver/urdf/wtb_car.xacro), [include/mqtt_client.hpp](src/wtb_car_driver/include/mqtt_client.hpp)

---

## 5. lidar_ros2 — 镭神激光雷达驱动

- **框架**：ROS2 + rclcpp (C++)
- **子包**：
  - `lslidar_msgs` — 自定义消息（LslidarScan, LslidarPoint, LslidarPacket）和服务（DevPort, MotorControl, MotorSpeed, DataIp, DataPort 等）
  - `lslidar_driver` — 驱动核心，解析雷达原始数据输出 `sensor_msgs/PointCloud2`

- **数据流**：
  1. `lslidar_driver` 从雷达接收原始数据包
  2. 解码后发布 `/point_cloud_raw`（3D 点云）
  3. `pointcloud_to_laserscan` 节点将 3D 点云投影为 2D 扫描，发布 `/scan`
  4. `/scan` 供 Cartographer SLAM 使用

- **主要文件**：[lslidar_driver/src/lslidar_driver_node.cpp](src/lidar_ros2/lslidar_ros/lslidar_driver/src/lslidar_driver_node.cpp)

---

## 6. my_cartographer — SLAM 建图

- **框架**：Google Cartographer ROS2，Lua 配置
- **依赖**：cartographer_ros

### SLAM 配置

- **2D SLAM 模式**
- **坐标系设定**：
  - `map_frame` = "map"（全局地图）
  - `tracking_frame` = "base_footprint"（跟踪框架）
  - `published_frame` = "odom"（发布参考）
- **不使用 IMU**，依赖 **EKF 融合后里程计**（`/ekf_odom`）+ 激光
- **激光范围**：0.5m ~ 30m
- **回环检测**：匹配分数 ≥ 0.60，最大检测距离 40m
- **后端优化**：Huber 核尺度 100，最大迭代 50 次，启用非单调步骤
- **运动滤波器**：角度 > 17° 或距离 > 0.2m 才触发处理

### 配置变体

- [cartographer.lua](src/my_cartographer/config/cartographer.lua) — 主配置（轮式里程计 + 激光）
- [lidar_cartographer.lua](src/my_cartographer/config/lidar_cartographer.lua) — 纯激光版本
- [localization_2d.lua](src/my_cartographer/config/localization_2d.lua) — 纯定位模式

### 启动文件

[cartographerAll.launch.py](src/my_cartographer/launch/cartographerAll.launch.py) 同时启动 Cartographer 节点、占用栅格发布节点、底盘驱动以及 RViz2。

---

## 7. my_navigation2 — 自主导航

- **框架**：Nav2 (ROS2 Navigation2)，Python
- **依赖**：rclcpp, nav2_bringup

### 功能组件

| 组件 | 文件 | 功能 |
|------|------|------|
| Nav2 启动 | [bringup_launch.py](src/my_navigation2/launch/bringup_launch.py) | 集成 SLAM/定位 + 导航栈（全局/局部规划器 + 控制器） |
| Qt 导航 GUI | [nav2_qt.py](src/my_navigation2/scripts/nav2_qt.py) | PyQt5 图形界面：TF 获取车辆位置、记录起终点、发起导航、保存/加载导航点 |
| 红绿灯控制 | [trafficLightControl.py](src/my_navigation2/scripts/trafficLightControl.py) | 交通信号灯响应逻辑（与 Nav2 集成） |
| 地图转换 | [pgm_to_fake_pcd.py](src/my_navigation2/scripts/pgm_to_fake_pcd.py) | PGM 地图转伪 PCD 格式 |

### Nav2 参数配置

提供多个 YAML 参数文件：`wtb_nav2_params.yaml`（主配置）、`eisa_nav2_params.yaml`（Eisa 平台）、`wheeltec_nav2_params.yaml`（WheelTec 底盘）。

---

# 第二部分：Alicia-M 机械臂

Alicia-M 是 **Synria Robotics** 制造的 **6-DOF 串联关节式机械臂 + 平行夹爪**，通过自定义串口协议通信。本工作区包含其完整的 ROS2 控制链：硬件驱动(ros2_control)、MoveIt2 运动规划、手眼标定、以及 6D 抓取模块(开发中)。

## 8. alicia_m_descriptions — 机械臂 URDF 模型与可视化

- **框架**：ROS2 + ament_cmake（纯 URDF/Mesh/Launch 安装）
- **依赖**：robot_state_publisher, joint_state_publisher, rviz2, tf2_ros, xacro

### 运动学链（所有关节为 revolute，除夹爪外）

| 关节 | 父连杆 | 子连杆 | 轴 | 限位 (rad) |
|------|--------|--------|-----|-----------|
| joint1 | base_link | link1 | Z | [-2.75, 2.75] |
| joint2 | link1 | link2 | Z | [-3.14, 0] |
| joint3 | link2 | link3 | -Z | [-3.14, 0] |
| joint4 | link3 | link4 | ~Z | [-1.57, 1.57] |
| joint5 | link4 | link5 | Z | [-1.57, 1.57] |
| joint6 | link5 | link6 | ~Z | [-2.75, 2.75] |
| left_finger | link6 | link7 | 直线(prismatic) | [-0.05, 0.0] m |
| right_finger | link6 | link8 | 直线(prismatic, mimics left_finger × -1.0) | [0.0, 0.05] m |
| tool0_fixed | link6 | tool0 | 固定 | — |

- SolidWorks 导出的 STL 网格（10 个，约 0.05~2.54 kg/连杆）
- 所有关节力矩限制 5 Nm，速度限制 10 rad/s
- 含 MuJoCo 兼容标签
- 含 RViz 可视化启动文件 `display.launch.py`

**主要文件**：[urdf/Alicia_M_v1_1/Alicia_M_v1_1_follower.urdf](src/Alicia-M-ROS2/alicia_m_descriptions/urdf/Alicia_M_v1_1/Alicia_M_v1_1_follower.urdf), [launch/display.launch.py](src/Alicia-M-ROS2/alicia_m_descriptions/launch/display.launch.py)

---

## 9. alicia_m_driver — 硬件接口插件（ros2_control）

- **框架**：ROS2 + rclcpp + hardware_interface + pluginlib + rclcpp_lifecycle
- **依赖**：rclcpp, hardware_interface, pluginlib, rclcpp_lifecycle

这是整个机械臂控制链中最关键的包，实现了 `hardware_interface::SystemInterface` 的 ros2_control 硬件插件，通过串口 UART 协议与 Alicia-M 微控制器通信。

### 架构（3 个 C++ 源文件）

**A. 串口层** (`serial_port.cpp`) — POSIX termios 非阻塞串口 I/O：
- `O_RDWR | O_NOCTTY | O_NONBLOCK` 方式打开设备
- 默认 1,000,000 bps，8N1，无流控，raw 模式
- 非阻塞读取 + select() 超时；阻塞写入 + EAGAIN 重试 + tcdrain()

**B. 协议层** (`protocol.cpp`) — Synria 自定义二进制协议：
- **帧格式**：`[0xAA 帧头][CMD][FUNC][LEN][...数据...][CRC][0xFF 帧尾]`
- **CRC**：CRC-32（多项式 0xEDB88320），取低 8 位
- **命令**：
  - `CMD_JOINT` (0x06) — 关节位置查询和控制
  - `CMD_ENABLE` (0x09) — 电机使能/失能
  - `CMD_TORQUE` (0x05) — 力矩锁定
  - `CMD_MOTOR_PARAM` (0x11) — 控制模式切换
  - `CMD_VERSION` (0x01) — 版本查询
  - `CMD_ERROR` (0xEE) — 错误报告

**两种控制模式**：

| 模式 | 说明 | 每个电机数据 |
|------|------|-------------|
| **PV 模式** (0x02) | 位置+速度控制 | 4 字节：位置 16bit ([-12.5,12.5] rad) + 速度 12bit ([-10,10] rad/s) |
| **MIT 模式** (0x01) | 位置控制 + Kp/Kd PID 参数 | 初始化 10 字节，运行时 2 字节。大关节(1-3) Kp=50/Kd=2，小关节(4-6+夹爪) Kp=20/Kd=1 |

**C. ros2_control 硬件接口层** (`alicia_m_system.cpp`) — `AliciaHardwareInterface`：
- 生命周期管理：`on_init` → `on_configure` → `on_activate` → `on_deactivate` → `on_cleanup`
- **read() 周期**：发送关节查询帧 → 非阻塞串行读取 → 解析反馈 → 速度估计 → 夹爪"echo mode"检测
- **write() 周期**：根据 control_mode 发送 PV 或 MIT 控制帧
- **夹爪反馈跟踪**：若硬件夹爪反馈超过 50 周期(0.5s)无新数据，切换到"echo mode"（位置向指令插值，0.05 m/s）
- **导出接口**：7 个关节(6 臂 + left_finger)的 position + velocity 状态接口；position 命令接口
- 以 pluginlib 类注册：`alicia_m_driver::AliciaHardwareInterface`

- **主要文件**：[src/alicia_m_system.cpp](src/Alicia-M-ROS2/alicia_m_driver/src/alicia_m_system.cpp), [src/protocol.cpp](src/Alicia-M-ROS2/alicia_m_driver/src/protocol.cpp), [src/serial_port.cpp](src/Alicia-M-ROS2/alicia_m_driver/src/serial_port.cpp)

---

## 10. alicia_m_moveit_config — MoveIt2 运动规划配置

- **框架**：MoveIt2 (Setup Assistant 生成) + Python launch
- **依赖**：moveit_ros_move_group, moveit_kinematics, moveit_planners, moveit_simple_controller_manager, alicia_m_descriptions, xacro

### 关键配置

**SRDF** (语义描述)：
- **规划组 "arm"**：链 `base_link` → `tool0`（joint1~6）
- **规划组 "gripper"**：link7, link8, tool0
- **末端执行器 "gripper"**：父 link6，组 gripper
- **虚拟关节**：固定 joint world → base_link
- **命名姿态**：home（全零）、open（手指 0）、close（手指 -0.05）
- **自碰撞矩阵**：相邻链接禁用，部分非相邻对标记为 Never/User

**运动学** (`kinematics.yaml`)：
- 求解器：KDL kinematics plugin
- 搜索分辨率 0.005，超时 0.005s

**控制器** (`moveit_controllers.yaml`)：
- `moveit_simple_controller_manager/MoveItSimpleControllerManager`
- `arm_controller`：FollowJointTrajectory 动作 (`/arm_controller/follow_joint_trajectory`)
- `gripper_controller`：GripperCommand 动作 (`/gripper_controller/gripper_cmd`)

**Pilz 笛卡尔限制**：最大平移速度 1.0 m/s，最大平移加速度 2.25 m/s²

**ros2_control Xacro** (`alicia_m_v1_1_follower.ros2_control.xacro`)：
- 真实硬件时（plugin != mock_components）：传递串口参数
- 映射 7 个关节的位置命令接口 + 位置/速度状态接口

**ros2_controllers.yaml**：
- 控制器更新率：100 Hz
- 三个控制器：`arm_controller` (JointTrajectoryController), `gripper_controller` (GripperActionController), `joint_state_broadcaster`

- **主要文件**：[config/alicia_m_v1_1_follower.srdf](src/Alicia-M-ROS2/alicia_m_moveit_config/config/alicia_m_v1_1_follower.srdf), [config/kinematics.yaml](src/Alicia-M-ROS2/alicia_m_moveit_config/config/kinematics.yaml), [config/alicia_m_v1_1_follower.ros2_control.xacro](src/Alicia-M-ROS2/alicia_m_moveit_config/config/alicia_m_v1_1_follower.ros2_control.xacro)

---

## 11. alicia_m_bringup — 启动文件集

- **框架**：ROS2 + ament_cmake（纯 launch + config 安装）
- **依赖**：alicia_m_descriptions, alicia_m_moveit_config, alicia_m_driver, moveit_ros_move_group, controller_manager, joint_trajectory_controller

三个启动模式：

| 启动文件 | 用途 | 包含内容 |
|----------|------|----------|
| `hardware.launch.py` | 硬件控制（无 MoveIt） | robot_state_publisher, ros2_control_node, 3 控制器(延迟生成: joint_state_broadcaster t+3s, arm_controller t+4s, gripper_controller t+5s) |
| `moveit_hardware.launch.py` | **推荐**：真实硬件 + MoveIt2 | 上述全部 + move_group + RViz2(MoveIt) + 静态 TF world→base_link |
| `moveit_sim.launch.py` | 仿真（mock 硬件） | 同 moveit_hardware，但 ros2_control 插件替换为 mock_components/GenericSystem |

可配置参数：`serial_port` (默认 `/dev/ttyACM0`), `control_mode` (`pv` 或 `mit`), `baudrate` (默认 1000000), `default_speed`, `mit_kp`, `mit_kd`。

- **主要文件**：[launch/moveit_hardware.launch.py](src/Alicia-M-ROS2/alicia_m_bringup/launch/moveit_hardware.launch.py), [config/ros2_controllers.yaml](src/Alicia-M-ROS2/alicia_m_bringup/config/ros2_controllers.yaml)

---

## 12. alicia_m_calibration — 手眼标定

- **框架**：Python (rclpy) + OpenCV
- **依赖**：rclpy, sensor_msgs, geometry_msgs, tf2_ros, cv_bridge, image_transport, control_msgs, moveit_msgs, python3-opencv, python3-scipy, python3-yaml

使用 ArUco 码 + Intel RealSense D405 进行 eye-in-hand 标定，计算 `tool0 → camera_link` 变换。

**工作流程**：
1. 控制机械臂依次移动到 20 个预设标定位姿
2. 在每个位姿检测 ArUco 标记（10 帧去噪，需要 5 帧稳定后用中值滤波）
3. 从 TF 记录 (R_gripper2base, t_gripper2base)
4. 通过 OpenCV PnP 计算 (R_target2cam, t_target2cam)
5. 使用 OpenCV `calibrateHandEye`（默认 Daniilidis 方法）计算眼在手外参
6. 结果保存到 `hand_eye_calibration_result.yaml`

**已存标定结果**（2026 年 3 月，15 个样本，Daniilidis 方法）：
- `camera_link` 相对 `tool0` 偏移 ≈ [-0.057, 0.010, -0.095] 米
- 旋转 ≈ [-30.3, -0.65, -83.8] 度 (Euler XYZ)

**辅助节点**：
- `aruco_detector.py` — 单独 ArUco 检测，发布 TF `camera_color_optical_frame → aruco_marker_frame`
- `verify_calibration.launch.py` — 加载标定结果，发布静态 TF `tool0 → camera_link`，用于目视验证

- **主要文件**：[scripts/hand_eye_calibration.py](src/Alicia-M-ROS2/alicia_m_calibration/scripts/hand_eye_calibration.py), [scripts/aruco_detector.py](src/Alicia-M-ROS2/alicia_m_calibration/scripts/aruco_detector.py), [config/hand_eye_calibration_result.yaml](src/Alicia-M-ROS2/alicia_m_calibration/config/hand_eye_calibration_result.yaml)

---

## 13. alicia_m_grasp_6d — 6D 抓取模块（开发中）

- **状态**：开发中（无 package.xml，无正式安装）
- **依赖**：FoundationStereo, GraspGen (NVIDIA 2025), SAM2

包含的脚本（未安装）：

| 脚本 | 功能 |
|------|------|
| `d405_foundationstereo.py` | FoundationStereo 深度估计 |
| `d405_graspgen.py` | GraspGen 抓取生成 |
| `d405_sam2.py` | SAM2 分割 |
| `d405_ros_bridge.py` | ROS 桥接节点 |
| `d405_execution.py` | 抓取执行 |

- **主要文件**：[d405_execution.py](src/Alicia-M-ROS2/alicia_m_grasp_6d/d405_execution.py), [d405_graspgen.py](src/Alicia-M-ROS2/alicia_m_grasp_6d/d405_graspgen.py)

---

## 14. examples — 示例脚本

`01_moveit_pick_and_place.py` — 完整的 pick-and-place 示例：
1. 回 HOME（全零）
2. 开夹爪
3. 移动到位置 A 上方
4. 下降到位置 A
5. 闭夹爪（抓取）
6. 抬起到位置 A 上方
7. 回 HOME
8. 移动到位置 B 上方
9. 下降到位置 B
10. 开夹爪（放置）
11. 抬起到 HOME

这是官方示例的独立说明；项目任务链不采用“规划失败后直接发送备用轨迹”的回退。项目统一通过 `wvcsc_arm_task` 的 MoveIt2 适配层执行，并由 `trajectory_retime_server` 对笛卡尔轨迹做一次重定时和合法性校验。

---

# 第三部分：仿真与集成

## 15. wvcsc_description — 复合机器人统一模型

- **框架**：ROS2 + ament_cmake（URDF/XACRO 安装）+ Gazebo Classic 插件
- **依赖**：wtb_car_driver, alicia_m_descriptions, alicia_m_moveit_config, gazebo_ros, gazebo_plugins, gazebo_ros2_control, robot_state_publisher, xacro

**功能**：将无人车底盘 + Alicia-M 机械臂组装为统一的 `wvcsc_utb_alicia` 复合机器人 XACRO 模型。

### 模型组合方式

通过 XACRO `xacro:include` 组合三个子模型：
1. [wtb_car.xacro](src/wtb_car_driver/urdf/wtb_car.xacro) — 阿克曼车体（base_link, 轮子, 转向关节, laser）
2. [Alicia_M_v1_1_follower.urdf](src/Alicia-M-ROS2/alicia_m_descriptions/urdf/Alicia_M_v1_1/Alicia_M_v1_1_follower.urdf) — 机械臂运动学
3. [alicia_m_v1_1_follower.ros2_control.xacro](src/Alicia-M-ROS2/alicia_m_moveit_config/config/alicia_m_v1_1_follower.ros2_control.xacro) — ros2_control 硬件接口

### 集成安装

- `arm_mount_link` — 0.24×0.24×0.12m，5kg，固定 joint `arm_mount_joint` 在 `base_link` 上方 0.56m
- `alicia_mount_joint` — 固定 joint，在 `arm_mount_link` 上方 0.06m，连接 `alicia_base_link`

### 可配置参数

| XACRO 参数 | 默认值 | 说明 |
|-----------|--------|------|
| `enable_arm_control` | `true` | 是否包含机械臂 ros2_control |
| `enable_ackermann` | `true` | 是否包含车辆关节状态发布插件 |
| `enable_gazebo_ros2_control` | `false` | 是否加载 Gazebo ros2_control 插件 |
| `ros2_control_plugin` | `mock_components/GenericSystem` | 硬件接口插件（仿真时改为 gazebo_ros2_control/GazeboSystem）|
| `serial_port` | `/dev/ttyACM0` | 串口设备 |
| `baudrate` | `1000000` | 串口波特率 |
| `control_mode` | `pv` | 控制模式 |

### Gazebo 传感器/插件（XACRO 中定义）

| 传感器/插件 | 详情 |
|------------|------|
| **激光雷达** | `libgazebo_ros_ray_sensor.so`，720 采样，范围 0.15~20m，10Hz，发布 `/scan`，帧 `laser` |
| **车辆关节状态** | `libgazebo_ros_joint_state_publisher.so`，50Hz，6 个阿克曼关节 |
| **ros2_control** | `libgazebo_ros2_control.so`，机械臂控制（条件启用） |

- **主要文件**：[urdf/wvcsc_utb_alicia.urdf.xacro](src/wvcsc_description/urdf/wvcsc_utb_alicia.urdf.xacro), [config/ros2_controllers.yaml](src/wvcsc_description/config/ros2_controllers.yaml)

---

## 16. wvcsc_simulation — 顶层仿真编排

- **框架**：Python (rclpy) + Gazebo Classic
- **依赖**：alicia_m_moveit_config, controller_manager, gazebo_ros, gazebo_ros2_control, gazebo_msgs, moveit_ros_move_group, nav2_bringup, trajectory_retime_server, wvcsc_description, wvcsc_arm_task

**功能**：顶层仿真启动包，提供 Gazebo 仿真 + Nav2 + MoveIt2 + 喷雾任务的完整编排。

### 主启动文件 `system_sim.launch.py`

启动参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_nav2` | `true` | 启用 Nav2 导航；机械臂基线回归可显式设为 `false` |
| `use_rviz` | `true` | 启动 RViz2；无图形环境可显式设为 `false` |
| `enable_arm_control` | `true` | 启用机械臂 ros2_control |

启动序列：
1. 设置 `GAZEBO_MODEL_PATH`
2. 启动 **Gazebo** 加载 `orchard.world`
3. 启动 **robot_state_publisher**（处理 XACRO → robot_description）
4. 发布静态 TF `world → odom`
5. 加载 **Move Group**（MoveIt2，含 OMPL 规划管道），SRDF 运行时 patch（基准改为 `alicia_base_link`，移除虚拟关节）
6. 在 t+3s, t+4s, t+5s 分别生成 joint_state_broadcaster, arm_controller, gripper_controller
7. 启动 `trajectory_retime_server`、`wvcsc_motion_control` 和 `wvcsc_spray_task`
8. 实体生成完成后：等 5s → 取消 Gazebo 暂停 → 启动 AckermannSim
9. 条件启动 Nav2（含 orchard 地图，WTB 参数）
10. 条件启动 RViz2

### AckermannSim 节点 (`ackermann_sim.py`)

自定义车辆运动学仿真节点：

| ROS 接口 | 类型 | 详情 |
|----------|------|------|
| `cmd_vel` 订阅 | `geometry_msgs/Twist` | 接收速度指令 |
| `/odom` 发布 | `nav_msgs/Odometry` | 地面真值里程计 |
| `/ekf_odom` 发布 | `nav_msgs/Odometry` | EKF 输入副本 |
| TF 广播 | `odom → base_footprint` | 车辆位姿 TF |
| Gazebo 服务调用 | `/set_entity_state` | 异步移动 Gazebo 机器人模型 |

**行为**：
- 20 Hz 更新率
- 0.5s 指令超时（停止车辆）
- 速度/航向角速度钳制 [-0.8, 0.8]
- 阿克曼积分：`yaw += yaw_rate * dt; x += speed * cos(yaw) * dt; y += speed * sin(yaw) * dt`
- 通过异步 `/set_entity_state` 传送 Gazebo 模型位姿

### Orchard 仿真世界 (`orchard.world`)

SDF 1.6 格式：
- 绿色地面 100m×100m
- 两个树（圆柱体，半径 0.25m，高 1.5m），位于 (3,2) 和 (5,-2)
- 终端障碍墙（0.3×4×1m）在 x=7m
- 物理引擎：ODE，实时更新率 1000
- `gazebo_ros_state` 插件 50Hz

### 地图

`orchard.yaml` — 分辨率 0.5m/pixel，三值模式，地图原点 (-10,-10)。

- **主要文件**：[launch/system_sim.launch.py](src/wvcsc_simulation/launch/system_sim.launch.py), [scripts/ackermann_sim.py](src/wvcsc_simulation/scripts/ackermann_sim.py), [worlds/orchard.world](src/wvcsc_simulation/worlds/orchard.world)

---

## 17. wvcsc_arm_task — 机械臂 MoveIt 适配与喷雾任务

- **框架**：Python (rclpy, ament_python)
- **依赖**：`pymoveit2` 4.2.0、MoveIt2、control_msgs、rclpy、trajectory_retime_server、std_srvs

**功能**：在项目代码中提供 Alicia-M 专用的轻量 MoveIt 适配层、运动锁定控制和模拟喷洒流程；不修改官方 `pymoveit2` 源码。

| ROS 接口 | 类型 | 详情 |
|----------|------|------|
| 节点 | — | `wvcsc_spray_task` |
| `/arm/execute_spray` 服务 | `std_srvs/Trigger`（当前过渡接口） | 触发观察→模拟喷洒→HOME；多目标编排前需增加完成结果语义 |
| `/motion_control/command` | `std_msgs/String` | `stop`、`reset`、`resume` |
| `/trajectory_execution_event` | `std_msgs/String` | 向 MoveIt 执行链发布 `stop` |
| `/retime_trajectory` | `trajectory_retime_server/srv/RetimeTrajectory` | 笛卡尔规划的唯一重定时入口 |
| MoveIt 执行 | `pymoveit2` → `move_group` → `arm_controller` | 普通关节/位姿轨迹不重复重定时；笛卡尔轨迹失败则禁止执行 |

预定义姿态（6 关节，弧度）：

| 姿态 | joint1 | joint2 | joint3 | joint4 | joint5 | joint6 |
|------|--------|--------|--------|--------|--------|--------|
| HOME | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| OBSERVE_LEFT | 0.65 | -1.35 | -1.05 | 0.0 | -0.75 | 0.65 |
| OBSERVE_RIGHT | -0.65 | -1.35 | -1.05 | 0.0 | -0.75 | -0.65 |

**行为序列**：
1. 移动到观察/喷雾姿态（根据 `spray_side` 参数选择左/右）
2. 模拟喷雾（sleep `spray_duration` 秒，默认 2.0s）
3. 通过 MoveIt 规划回到全零 HOME
4. `stop`/`reset` 取消当前运动；`reset` 先停、开夹爪、回 HOME，失败保持锁定
5. `busy` 标志防止并发；`resume` 只解除锁定，不恢复已取消轨迹

- **主要文件**：[wvcsc_arm_task/spray_task.py](src/wvcsc_arm_task/wvcsc_arm_task/spray_task.py)

---

## 18. wvcsc_vehicle_sim

- **状态**：空占位目录（无 package.xml，无源代码）
- 可能是计划中的高级车辆动力学仿真器（替代 `ackermann_sim.py`）

---

# 系统架构

## 包依赖关系图

```
                    ┌──────────────────────┐
                    │  wvcsc_simulation    │ (顶层仿真编排)
                    │  - system_sim.launch │
                    │  - ackermann_sim.py  │
                    │  - orchard.world     │
                    └────┬─────┬─────┬─────┘
                         │     │     │
              ┌──────────┘     │     └──────────────┐
              ▼                ▼                    ▼
    ┌─────────────────┐  ┌──────────────┐  ┌────────────────┐
    │ wvcsc_description│  │wvcsc_arm_task│  │  nav2_bringup  │ (Nav2)
    │ (复合机器人模型)  │  │(喷雾任务)    │  └────────────────┘
    └──┬───────────┬──┘  └──────────────┘
       │           │
       ▼           ▼
  ┌─────────┐  ┌──────────────────────┐
  │wtb_car  │  │ alicia_m_moveit_config│ (MoveIt2)
  │_driver  │  └──────────┬───────────┘
  └────┬────┘             │
       │                  ▼
       │        ┌─────────────────┐
       │        │alicia_m_descriptions│ (机械臂URDF)
       │        └────────┬────────┘
       │                 │
       │                 ▼
       │        ┌─────────────────┐
       │        │ alicia_m_driver │ (ros2_control硬件插件)
       │        └────────┬────────┘
       │                 │
       │                 ▼
       │        ┌─────────────────┐
       │        │  serial (库)    │
       │        └─────────────────┘
       │
       ▼
  ┌──────────┐    ┌───────────────┐    ┌──────────────┐
  │ can_bridge│   │fdilink_ahrs   │    │  lidar_ros2  │
  └─────┬─────┘   └───────┬───────┘    └──────┬───────┘
        │                 │                   │
        ▼                 ▼                   ▼
  ┌──────────┐    ┌───────────────┐    ┌──────────────┐
  │libcontrol│    │  serial (库)  │    │ lslidar_msgs │
  │can.so    │    └───────────────┘    └──────────────┘
  └──────────┘
```

## 完整仿真启动流程

```
system_sim.launch.py
├── Gazebo (orchard.world, 暂停启动)
├── robot_state_publisher (XACRO复合模型 → /robot_description)
├── static TF: world → odom
├── spawn_entity (从 /robot_description 生成机器人)
├── Move Group (MoveIt2, OMPL + Pilz, SRDF运行时patch)
├── [t+3s] joint_state_broadcaster
├── [t+4s] arm_controller (JointTrajectoryController)
├── [t+5s] gripper_controller (GripperActionController)
├── trajectory_retime_server (笛卡尔轨迹重定时与校验)
├── wvcsc_motion_control (stop/reset/resume 锁定控制)
├── wvcsc_spray_task (MoveIt 模拟喷洒任务)
├── [post-spawn+5s] AckermannSim (车辆运动学仿真, 20Hz)
│   ├── 订阅 /cmd_vel
│   ├── 发布 /odom, /ekf_odom
│   ├── 发布 TF odom→base_footprint
│   └── 调用 /set_entity_state (Gazebo模型移动)
├── [条件] Nav2 (orchard地图, wtb参数, autostart)
│   ├── localization (AMCL)
│   └── navigation (全局+局部规划器, 控制器)
└── [条件] RViz2 (MoveIt配置)
```

48 小时内将在上述流程前增加 `wvcsc_uav_gateway`（Mock 目标发布）和 `wvcsc_mission_manager`（Nav2→机械臂任务编排）；这两个节点当前尚未进入工作区，不能把规划中的接口描述误认为已实现。

## 真实硬件启动流程

```
start_wtb_car_fdimu.launch.py
├── can_bridge               CAN总线桥接 (硬件 ↔ ROS2)
├── imu_launch               FDILink AHRS/IMU/GPS 驱动
├── joint_state_publisher    关节状态发布 (TF树)
├── robot_state_publisher    URDF机器人模型发布 (TF树)
├── wtb_car                  底盘驱动 + 轮式里程计
├── lidar_launch             镭神激光雷达 (3D点云)
├── pointcloud_to_laserscan  3D点云 → 2D激光 /scan
├── ekf_node (robot_localization)
│   ├── 输入: /car_odom (轮式里程计) + /imu (IMU)
│   └── 输出: /ekf_odom (融合里程计) + odom→base_footprint TF
└── rviz2                    3D 可视化
```

## 复合机器人 TF 坐标变换树

```
map ──(Cartographer)──▶ odom ──(EKF)──▶ base_footprint ──(fixed)──▶ base_link
                                                                       │
                               ┌───────────────────────────────────────┬┴──────────────────┐
                               │                                       │                   │
                               ▼                                       ▼                   ▼
                         arm_mount_link                             laser           left_wheel
                         (0,0,0.56m)                                                right_wheel
                               │                                                       ...
                               ▼
                        alicia_base_link ──▶ link1 ──▶ link2 ──▶ link3 ──▶ link4 ──▶ link5 ──▶ link6
                                                                                              │
                                                                              ┌───────────────┴──┐
                                                                              ▼                  ▼
                                                                           tool0              link7
                                                                           (固定关节)        (left_finger)
                                                                              │                  │
                                                                              ▼                  ▼
                                                                         camera_link        link8
                                                                         (标定结果)     (right_finger,
                                                                                        mimics left_finger)
```

---

## 数据流总览（完整系统）

```
┌──────────────────────┐
│      仿真/真实        │
│                      │
│ Gazebo orchard.world  │──▶ /scan (720采样, 10Hz)
│ AckermannSim (20Hz)   │──▶ /odom, /ekf_odom, TF odom→base_footprint
│ /cmd_vel → AckermannSim (Gazebo 中移动车辆模型)       │
│ /cmd_vel → wtb_car (真实硬件 CAN 指令)                 │
└──────────────────────┘

┌──────────────────────┐     ┌──────────────────────┐
│ Mock/Replay/Live UAV  │────▶│ 任务管理器            │
│ 病树坐标与任务列表    │     │ 坐标校验/停靠位姿/队列 │
└──────────────────────┘     └──────────┬───────────┘
                                        │ NavigateToPose
                                        ▼
                                  ┌──────────────┐
                                  │ Nav2 → /cmd_vel│
                                  └──────┬───────┘
                                         │ 到达并停稳
                                         ▼
                                  ┌──────────────┐
                                  │ Alicia-M喷洒  │
                                  │ MoveIt→HOME  │
                                  └──────────────┘

┌──────────────────────┐     ┌──────────────────────┐     ┌──────────────────────┐
│   传感器 (真实)       │     │      融合层           │     │    SLAM/导航          │
│  FDILink → /imu      │────▶│ robot_localization   │────▶│ Cartographer         │
│  底盘CAN → /car_odom  │     │ EKF → /ekf_odom      │     │ /ekf_odom + /scan    │
│  镭神LiDAR → /scan    │     └──────────────────────┘     │ → /map + 位姿估计    │
└──────────────────────┘                                   └──────────┬───────────┘
                                                                     │
                                                                     ▼
                                                          ┌──────────────────────┐
                                                          │ Nav2 导航栈          │
                                                          │ 全局/局部规划 + 控制 │
                                                          └──────────┬───────────┘
                                                                     │ /cmd_vel
                                                                     ▼
                                                          ┌──────────────────────┐
                                                          │ 底盘 CAN 控制        │
                                                          │ (线速度 + 转向角)    │
                                                          └──────────────────────┘

┌──────────────────────┐
│   机械臂控制          │
│                      │
│ MoveIt2 + OMPL       │──▶ /arm_controller/follow_joint_trajectory
│ wvcsc_spray_task     │──▶ 喷雾任务序列 → 同上 action
│ hand_eye_calibration │──▶ tool0 → camera_link 外参标定
│ alicia_m_bringup     │──▶ ros2_control_node → serial UART → 微控制器
│                      │     ├── arm_controller (关节 1-6 位置控制, 100Hz)
│                      │     └── gripper_controller (夹爪位置控制)
└──────────────────────┘
```

---

## 关键特性总览

### 无人车
1. **多传感器融合**：轮式里程计 + IMU + GPS 通过 EKF 融合
2. **阿克曼运动学**：完整的阿克曼底盘里程计模型，支持参数在线校准
3. **CAN 总线通信**：双通道 CAN 总线管理底盘控制和状态反馈，含 BCC 校验
4. **二进制协议解析**：FDILink 传感器自定义二进制帧协议，CRC8/CRC16 双重校验
5. **Cartographer 2D SLAM**：基于融合里程计和 2D 激光扫描的实时建图
6. **Nav2 自主导航**：完整的规划 + 控制 + 行为树导航栈
7. **PyQt5 图形界面**：可视化导航控制与状态监控
8. **Autoware 兼容**：启动文件和配置兼容 Autoware.universe 生态

### 机械臂
9. **ros2_control 硬件接口**：Synria 自定义二进制串口协议，PV/MIT 双控制模式
10. **MoveIt2 运动规划**：OMPL + Pilz Industrial Motion Planner，KDL 运动学
11. **手眼标定**：ArUco + RealSense D405 + OpenCV HandEye（Daniilidis 法）
12. **6D 抓取**（开发中）：FoundationStereo + GraspGen + SAM2

### 仿真
13. **Gazebo Classic 仿真**：自定义 orchard 世界，阿克曼车辆 + 机械臂复合模型
14. **车辆运动学仿真**：AckermannSim 节点 20Hz 阿克曼积分 + Gazebo 模型传送
15. **复合机器人模型**：WVCSC UTB 底盘 + Alicia-M 机械臂统一 XACRO

### 通用
16. **跨平台**：同时支持 x86_64 开发和 ARM aarch64 实车部署
17. **多线程执行**：底盘控制、CAN 桥接均使用多线程

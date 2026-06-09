# WVCSC Autoware 实车部署手册

本文档是 `WVCSC_S2Z_UTB_ARM` 实车部署唯一维护的主手册。覆盖建图、编译、启动、定位、规划控制和故障排查。

**基线环境：** Ubuntu 22.04 / ROS 2 Humble / Autoware `/home/eisa/autoware` / WVCSC `/home/eisa/WVCSC_S2Z_UTB_ARM`

---

## 1. 两条核心链路

WVCSC 分成两条独立链路，**不要混成一个总入口**：

```
建图链                         运行链
LIO-SAM                       wvcsc_autoware_bringup
LiDAR + IMU → pointcloud_map   map → localization → planning → control → chassis
```

---

## 2. 包结构总览

```
WVCSC_S2Z_UTB_ARM/src/
├── can_bridge/                     CAN ↔ ROS2 桥接
├── wtb_car_driver/                 底盘驱动 (Ackermann + CAN协议)
├── wvcsc_vehicle_interface/        Autoware控制 ↔ 底盘适配器
├── wvcsc_vehicle_launch/           车辆URDF + vehicle.launch
│   ├── wvcsc_vehicle_description/  车辆参数 + vehicle_info.param.yaml
│   └── wvcsc_vehicle_launch/       robot_state_publisher + vehicle_interface
├── wvcsc_sensor_kit_launch/        传感器驱动套件
│   ├── wvcsc_sensor_kit_description/  传感器URDF + 标定
│   ├── wvcsc_sensor_kit_launch/        sensing.launch.xml
│   └── common_sensor_launch/          共享 LiDAR/IMU 驱动入口
├── wvcsc_autoware_bringup/         顶层整合 + 定位 + Autoware适配
├── lio_sam/                        LIO-SAM 3D建图
├── lidar_ros2/lslidar_ros/        雷神LiDAR驱动
├── fdilink_ahrs_ROS2/              FDILink IMU驱动
├── serial/                         ROS2串口库
├── my_cartographer/                Cartographer 2D建图 (参考链)
└── my_navigation2/                 Nav2 2D导航 (参考链)
```

### 包职责速查

| 包 | 角色 | 是否正式运行链 |
|----|------|:---:|
| `can_bridge` | USB-CAN ↔ `can_msgs/Frame` | ✅ |
| `wtb_car_driver` | 底盘解码 + 里程计 + CAN命令发送 | ✅ |
| `wvcsc_vehicle_interface` | Autoware `control_cmd` → `twist_cmd` 适配 | ✅ |
| `wvcsc_vehicle_launch` | 车体URDF TF发布 | ✅ |
| `wvcsc_sensor_kit_launch` | LiDAR/IMU驱动入口 | ✅ |
| `wvcsc_autoware_bringup` | 硬件+定位+Autoware全栈bringup | ✅ |
| `lio_sam` | 3D点云地图生产 | ✅ 建图 |
| `my_cartographer` | 2D SLAM | 参考 |
| `my_navigation2` | 2D Nav2导航 | 参考 |

---

## 3. TF 树与传感器外参

### 3.1 TF 链（以 `wtb_car.xacro` 为基准）

```
map ──(static)──▶ odom ──(EKF)──▶ base_footprint ──(URDF)──▶ base_link
                                                               │
                                                    ┌──────────┤
                                                    ▼          ▼
                                          sensor_kit_base_link (零偏移层)
                                                    │
                                         ┌──────────┤
                                         ▼          ▼
                                       laser      gyro_link
                                    (0.36,0,0.50) (0,0,-0.40)
```

### 3.2 传感器外参

| 传感器 | 父frame | x(m) | y(m) | z(m) | roll | pitch | yaw |
|--------|---------|------|------|------|------|-------|-----|
| **laser** | sensor_kit_base_link | 0.36 | 0.0 | 0.50 | 0.0 | 0.0 | 0.0 |
| **gyro_link** | sensor_kit_base_link | 0.0 | 0.0 | -0.40 | 0.0 | 0.0 | 0.0 |

### 3.3 Frame 命名规则

| 正式frame | 禁止使用的旧名 |
|-----------|---------------|
| `laser` | `laser_link` |
| `gyro_link` | `imu_link` |
| — | `gnss_link` (已移除) |
| — | `navsat_link` (已移除) |

---

## 4. 数据流

```
┌─ 传感器 ───────────────────────────────────────────────────┐
│ LiDAR (UDP:192.168.1.200:2368) → lslidar_driver            │
│   → /sensing/lidar/pointcloud_raw (PointXYZIRT, laser)     │
│ IMU (/dev/FDI_IMU_GNSS) → fdilink_ahrs                     │
│   → /sensing/imu/tamagawa/imu_raw (gyro_link)              │
└────────────────────────────────────────────────────────────┘

┌─ 车辆 ─────────────────────────────────────────────────────┐
│ CAN → can_bridge → can_msgs/Frame                          │
│ can_msgs/Frame → wtb_car → /car_odom, /wtb_car_message    │
│                                                             │
│ 控制流:                                                     │
│ /control/command/control_cmd → wvcsc_vehicle_interface      │
│   → /twist_cmd → wtb_car → CAN → 底盘执行                  │
│                                                             │
│ 反馈流:                                                     │
│ /car_odom → wvcsc_vehicle_interface                         │
│   → /vehicle/status/velocity_status → Autoware             │
└────────────────────────────────────────────────────────────┘

┌─ Autoware ─────────────────────────────────────────────────┐
│ localization: NDT scan matcher + EKF pose fusion           │
│ planning: behavior_planner + motion_planner                │
│ control: pure_pursuit → /control/command/control_cmd       │
└────────────────────────────────────────────────────────────┘
```

---

## 5. 编译

### 5.1 环境准备

```bash
# 工控机上已安装 acados 到 ~/.local/acados
export CMAKE_PREFIX_PATH="/home/eisa/.local/acados:${CMAKE_PREFIX_PATH}"
export ACADOS_SOURCE_DIR="/home/eisa/.local/acados"
export LD_LIBRARY_PATH="/home/eisa/.local/acados/lib:${LD_LIBRARY_PATH}"

# LIO-SAM 依赖
sudo apt-get install -y ros-humble-gtsam
```

### 5.2 Autoware 编译

**工控机上必须限制并行度，否则桌面假死。** 建议在纯终端 / tmux / SSH 环境执行。

```bash
cd /home/eisa/autoware
source /opt/ros/humble/setup.bash

export CMAKE_PREFIX_PATH="/home/eisa/.local/acados:${CMAKE_PREFIX_PATH}"
export ACADOS_SOURCE_DIR="/home/eisa/.local/acados"
export LD_LIBRARY_PATH="/home/eisa/.local/acados/lib:${LD_LIBRARY_PATH}"
export MAKEFLAGS=-j2

colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 5.3 WVCSC 工作区编译

```bash
cd /home/eisa/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

---

## 6. LIO-SAM 建图

### 6.1 直播建图

```bash
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch lio_sam run_wvcsc_mapping.launch.py \
  launch_hardware:=true \
  launch_rviz:=false
```

TF 归属：
- `map → odom`：静态TF（建图launch发布）
- `odom → base_footprint`：EKF动态TF
- `base_footprint → base_link → laser/gyro_link`：robot_state_publisher静态TF

**建图技巧：**
- 启动后静止3-5秒让IMU初始化
- 缓慢直线行驶5-10米
- 绕场地走闭合路线（3-5分钟）
- 确保回到起点附近产生回环

### 6.2 建图前检查

```bash
ros2 topic hz /sensing/lidar/pointcloud_raw    # ~10Hz
ros2 topic hz /sensing/imu/tamagawa/imu_raw     # ~100Hz
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link gyro_link
```

重点确认：点云有 `ring` 和 `time` 字段，`frame_id` 为 `laser`。

### 6.3 保存地图

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: '/home/eisa/autoware_data/maps/wvcsc_lio_sam'}"
```

---

## 7. 硬件链启动

### 7.1 Launch 层次

```
hardware.launch.xml
├── vehicle_hardware.launch.xml      ← CAN + 底盘 + EKF + 车辆接口
│   ├── vehicle.launch.xml           ← URDF → robot_state_publisher
│   ├── can_bridge.launch.py
│   ├── wtb_car node
│   ├── EKF node (可选)
│   └── vehicle_interface.launch.xml
└── sensor_hardware.launch.xml       ← LiDAR + IMU
    └── sensing.launch.xml
        ├── lidar.launch.xml         ← lslidar_cx
        ├── imu.launch.xml           ← fdilink_ahrs
        └── vehicle_velocity_converter
```

### 7.2 单独启动硬件

```bash
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch wvcsc_autoware_bringup hardware.launch.xml
```

### 7.3 硬件链验证

```bash
# TF
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link gyro_link

# LiDAR
ros2 topic hz /sensing/lidar/pointcloud_raw
ros2 topic echo /sensing/lidar/pointcloud_raw --once

# IMU
ros2 topic hz /sensing/imu/tamagawa/imu_raw
ros2 topic echo /sensing/imu/tamagawa/imu_raw --once

# 底盘
ros2 topic echo /car_odom --once
ros2 topic echo /vehicle/status/velocity_status --once
```

---

## 8. 运行时启动

### 8.1 Hybrid模式（推荐首次联调）

**无感知，专注验证定位+规划+控制。**

```bash
# 清理旧进程
pkill -f "ros2 launch wvcsc_autoware_bringup"
pkill -f component_container
pkill -f rviz2
ros2 daemon stop; ros2 daemon start

# 启动
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch wvcsc_autoware_bringup hybrid_real_vehicle.launch.xml \
  map_path:=/home/eisa/autoware_data/maps/wvcsc_map \
  rviz:=true
```

启动模块：硬件 ✅ | 定位 ✅ | 规划 ✅ | 控制 ✅ | 感知 ❌

### 8.2 Full模式

**完整自动驾驶。需预先下载ML模型。**

```bash
ros2 launch wvcsc_autoware_bringup full_real_vehicle.launch.xml \
  map_path:=/home/eisa/autoware_data/maps/wvcsc_map \
  rviz:=true
```

启动模块：硬件 ✅ | 定位 ✅ | 规划 ✅ | 控制 ✅ | 感知 ✅

### 8.3 等效 planning_simulator → 实车

```bash
# 仿真
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/path/to/map vehicle_model:=wvcsc_vehicle sensor_model:=wvcsc_sensor_kit

# 等效实车
ros2 launch wvcsc_autoware_bringup full_real_vehicle.launch.xml \
  map_path:=/path/to/map
```

`vehicle_model`/`sensor_model` 在WVCSC bringup中已默认为对应值。

---

## 9. 运行时验证

### 9.1 定位链

```bash
ros2 topic hz /localization/pose_estimator/pose_with_covariance
ros2 topic hz /localization/kinematic_state
ros2 run tf2_ros tf2_echo map base_link
ros2 service list | grep trigger_node
# 期望: /localization/pose_estimator/trigger_node
#       /localization/pose_twist_fusion_filter/trigger_node
```

**NDT初始化：** WVCSC无GNSS，在RViz使用 `2D Pose Estimate` 手动给定初始位姿。

### 9.2 地图加载

```bash
ros2 topic list | grep '^/map'
ros2 topic info /map/pointcloud_map -v
```

### 9.3 控制链

```bash
ros2 topic echo /control/command/control_cmd --once
ros2 topic echo /twist_cmd --once
```

---

## 10. RViz 配置

当前使用 WVCSC 专用配置：`wvcsc_autoware_bringup/rviz/wvcsc_autoware.rviz`

此配置的 `TopDownOrtho` 初始视角已对准 `wvcsc_map` 中心附近 `(-27, 54)`。

如果地图已加载但RViz黑屏：
1. `Views → Current View` → `Type = TopDownOrtho` / `X≈-27` / `Y≈54` / `Scale≈180`
2. `Map → PointCloudMap` → `Size = 2` / `Alpha = 1.0` / `Color Transformer = FlatColor`

---

## 11. 启动顺序

```
1. 确认硬件上电 (网线/LiDAR/IMU/CAN)
2. hardware.launch.xml → 验证TF和话题
3. hybrid_real_vehicle.launch.xml → 手动初始化定位 → 验证规划控制
4. full_real_vehicle.launch.xml (感知就绪后)
```

---

## 12. 故障排查

### 12.1 LiDAR无数据

```bash
ros2 topic info /sensing/lidar/pointcloud_raw -v
ip addr                                    # 确认有192.168.1.x
ping 192.168.1.200                         # LiDAR IP
```

常见原因：网卡未配IP、雷达未上电、QoS不匹配。

### 12.2 `Waiting for IMU data ...`

LIO-SAM建图时出现此消息 → 已修复。`imageProjection.cpp` 现在兼容LSLiDAR的负值time约定。若仍出现，确认 `/sensing/imu/tamagawa/imu_raw` 有数据且frame_id为 `gyro_link`。

### 12.3 `package 'autoware_launch' not found`

Autoware工作区的 `autoware_launch` 未编译：

```bash
cd /home/eisa/autoware
colcon build --packages-up-to autoware_launch --symlink-install
```

### 12.4 缺少 ML 模型文件

```
No such file: '.../lidar_centerpoint/centerpoint_tiny_ml_package.param.yaml'
```

先跑 `hybrid_real_vehicle`（无感知），或下载模型：

```bash
mkdir -p /home/eisa/autoware_data/ml_models
cd /home/eisa/autoware_data/ml_models
git clone https://github.com/autowarefoundation/autoware_ml_data.git lidar_centerpoint
```

### 12.5 点云格式错误

```
The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC
```

确认NDT输入是 `/sensing/lidar/pointcloud_autoware`（经 `pointcloud_xyzirt_to_xyzirc_node` 转换），字段为 `x y z intensity return_type channel`。

### 12.6 Ctrl+Z 后无法重启

Ctrl+Z 挂起进程而非终止。清理：

```bash
kill %1 %2 %3 2>/dev/null
pkill -f "ros2 launch"
pkill -f rviz2
ros2 daemon stop; ros2 daemon start
```

### 12.7 编译假死

工控机编译 Autoware 必须限制资源：

```bash
MAKEFLAGS=-j2 colcon build --symlink-install --executor sequential
```

增大swap（`/dev/shm` 7.8G通常够用）：

```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### 12.8 `/tf has not received`

NDT未初始化前 `map → base_link` 不存在是正常的。先用 `2D Pose Estimate` 给初值。

### 12.9 `wtb_car` 刷屏 `Received 'start'`

旧版 `wvcsc_vehicle_interface` 重复发布 `/run_static`。已修复为仅在start/stop变化时发布。如仍出现，确认编译并source了最新版本。

---

## 13. 已知限制

| 限制 | 说明 |
|------|------|
| 无GNSS | 定位仅依赖 NDT + LiDAR + IMU + 轮速计 |
| 无自动初始化 | 需在 RViz 手动 `2D Pose Estimate` |
| ML 模型 | `full_real_vehicle` 需预先下载 lidar_centerpoint |
| C16 LiDAR | 点云较稀疏，建议建图时低速多角度覆盖 |
| 工控机资源 | 全量编译需低并行，swap 建议 ≥ 16G |

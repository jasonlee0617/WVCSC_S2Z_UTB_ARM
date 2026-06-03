# WVCSC Autoware 实车 Bringup 手册

本文档是 `WVCSC_S2Z_UTB_ARM` 的唯一实车部署主手册，目标是把两条链路彻底分开并讲清楚：

- 建图链：`LIO-SAM-ROS2`，只负责三维地图重建和导出 `pointcloud_map.pcd`
- 运行链：`wvcsc_autoware_bringup/full_real_vehicle.launch.xml`，负责地图加载、定位、感知、规划、控制、底盘执行

这也是当前最稳妥的工程路线：`LIO-SAM` 不直接接管 `full_real_vehicle.launch.xml` 的在线定位，而是先把正式地图产出来，再交给 Autoware 的定位链使用。

## 0. 环境准备

推荐每次都按下面顺序准备环境：

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source /home/robot/autoware/install/setup.bash
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

如果刚改过代码，先编译：

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source /home/robot/autoware/install/setup.bash
colcon build --symlink-install
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

建议先确认核心包都能被找到：

```bash
ros2 pkg prefix wvcsc_autoware_bringup
ros2 pkg prefix wvcsc_vehicle_launch
ros2 pkg prefix wvcsc_sensor_kit_launch
ros2 pkg prefix wvcsc_vehicle_interface
ros2 pkg prefix lio_sam
```

### 0.1 LIO-SAM 额外依赖

`LIO-SAM-ROS2` 在 Humble 下最容易卡住的是 `GTSAM`。当前工作区如果出现下面这类错误：

```text
Could not find a package configuration file provided by "GTSAM"
```

先安装：

```bash
sudo apt-get install -y ros-humble-gtsam
```

然后重新执行：

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source /home/robot/autoware/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## 1. 先建立正确心智模型：两条工作链

### 1.1 建图链

建图链使用：

```text
LIO-SAM-ROS2
├── launch/run_wvcsc_mapping.launch.py
└── launch/run_wvcsc_offline_mapping.launch.py
```

职责只有一个：

```text
LiDAR + IMU + 底盘辅助状态 -> 三维重建 -> pointcloud_map.pcd
```

它不是当前实车自动驾驶运行时入口。

### 1.2 运行链

运行链使用：

```text
wvcsc_autoware_bringup/full_real_vehicle.launch.xml
```

职责是：

```text
pointcloud_map.pcd + lanelet2_map.osm + map_projector_info.yaml
  -> localization
  -> perception
  -> planning
  -> control
  -> wvcsc_vehicle_interface
  -> wtb_car_driver
```

当前运行链保留 Autoware 规划层和控制层，不需要再额外补一个总入口。

## 2. `turn_on_wheeltec_robot.launch.py` 的具体作用

`turn_on_wheeltec_robot.launch.py` 本质上是旧 `wheeltec` 风格的基础硬件总入口。它做了 6 件事：

1. 通过 `base_serial.launch.py` 拉起底盘串口驱动。
2. 通过 `robot_mode_description.launch.py` 选择车体模型和描述文件。
3. 通过 `wheeltec_ekf.launch.py` 拉起 EKF。
4. 发布两个静态 TF：`base_footprint -> base_link`、`base_footprint -> gyro_link`。
5. 运行 `imu_filter_madgwick_node` 做 IMU 姿态后处理。
6. 运行 `joint_state_publisher`。

在 WVCSC 体系里，这些职责已经被本地化组件替代：

- `wtb_car_driver`
- `wvcsc_vehicle_launch/vehicle.launch.xml`
- `wvcsc_sensor_kit_launch/sensing.launch.xml`
- `wvcsc_autoware_bringup/hardware.launch.xml`
- `wvcsc_autoware_bringup/localization.launch.xml`

所以它现在更适合被当作：

```text
wheeltec 历史模板
```

而不是 WVCSC 长期依赖的正式入口。

## 3. 如何理解 LIO-SAM 从 wheeltec 风格迁移到 WVCSC 风格

当前已新增两项 WVCSC 化改造：

- `config/wvcsc_mapping_params.yaml`
- `launch/run_wvcsc_mapping.launch.py`
- `launch/run_wvcsc_offline_mapping.launch.py`

它们的目标是让 LIO-SAM 直接对接 WVCSC 当前硬件接口：

- 点云：`/sensing/lidar/pointcloud_raw`
- IMU：`/sensing/imu/tamagawa/imu_raw`
- GNSS：`/sensing/gnss/nav_sat_fix`
- LiDAR frame：`laser`
- 车体 frame：`base_footprint`

### 3.1 当前仍然保留但不建议再作为主入口的旧文件

下面这些旧 wheeltec 入口先保留作参考，但不建议继续作为主入口：

- `launch/run.launch.py`
- `launch/run_offline.launch.py`
- `launch/run_gnss.launch.py`
- `launch/run_offline_gnss.launch.py`
- `launch/run_usegnss.launch.py`
- `launch/include/turn_on_wheeltec_robot_lio.launch.py`
- `launch/include/wheeltec_sensors_liosam.launch.py`
- `config/wheeltec_params.yaml`

建议后续的使用习惯改成：

- 直播建图：`run_wvcsc_mapping.launch.py`
- 离线建图：`run_wvcsc_offline_mapping.launch.py`

## 4. LIO-SAM 建图前必须先核对的输入条件

### 4.1 点云字段

LIO-SAM 强依赖点云中包含：

- `ring`
- `time`

当前 WVCSC 雷神雷达驱动已经有较大概率满足这一点，因为 `lslidar_driver` 里定义了 `PointXYZIRT`，并且当前 Autoware sensing 话题已经统一为：

```text
/sensing/lidar/pointcloud_raw
```

建议你实车前一定检查一次字段：

```bash
ros2 topic echo /sensing/lidar/pointcloud_raw --once
```

如果后续发现 LIO-SAM 报错与字段不匹配，应优先检查：

- 是否真的带 `ring`
- `time` 是否是每一圈扫描内部的相对时间，而不是空值或绝对时间戳

### 4.2 IMU 频率与时间同步

LIO-SAM 对 IMU 要求比普通导航链更严格。建议至少确认：

```bash
ros2 topic hz /sensing/imu/tamagawa/imu_raw
```

同时关注：

- LiDAR 与 IMU 是否时间同步
- `gyro_link` 与 `laser` 外参是否匹配真实安装关系
- `wvcsc_mapping_params.yaml` 中的外参是否和你现在的实车标定一致

当前 `wvcsc_mapping_params.yaml` 是按照现有实车模型先给出的初始值：

```text
base_link -> laser = (0.36, 0.0, 0.50)
base_link -> gyro_link = (0.0, 0.0, -0.40)
```

所以 `extrinsicTrans` 目前写成的是 IMU 到 LiDAR 的相对平移近似值。真正上车建图前，建议你再结合实物安装关系做一次复核。

## 5. LIO-SAM 建图流程

### 5.1 直播建图

推荐命令：

```bash
ros2 launch lio_sam run_wvcsc_mapping.launch.py   launch_hardware:=true   launch_rviz:=false
```

它会做两件事：

1. 条件启动 `wvcsc_autoware_bringup/hardware.launch.xml`
2. 启动 LIO-SAM 四个核心节点：
   - `lio_sam_imuPreintegration`
   - `lio_sam_imageProjection`
   - `lio_sam_featureExtraction`
   - `lio_sam_mapOptimization`

建议建图时：

- 低速平稳行驶
- 先小范围闭环一圈
- 再逐步扩大区域
- 避免急转、急停、剧烈颠簸

### 5.2 离线建图

如果你已经录好了 bag，推荐命令：

```bash
ros2 launch lio_sam run_wvcsc_offline_mapping.launch.py   launch_rviz:=false
```

然后在另一个终端播放 bag：

```bash
ros2 bag play <your_bag_path>
```

这条链适合做：

- 参数细调
- 地图质量复盘
- 不上车的离线重建

### 5.3 保存地图

LIO-SAM 运行稳定后，可以调用保存服务：

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap
```

推荐显式指定目录和分辨率：

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '/home/robot/autoware_data/maps/wvcsc_lio_sam'}"
```

保存后建议立即检查：

```bash
pcl_viewer /home/robot/autoware_data/maps/wvcsc_lio_sam/GlobalMap.pcd
```

如果地图存在以下问题，需要优先回查时间同步和外参：

- 明显撕裂
- 墙体重复重影
- 上下跳动
- 转弯处扭曲

## 6. 正式地图资产如何落地给 Autoware

Autoware 正式地图目录建议统一为：

```text
/home/robot/autoware_data/maps/wvcsc_map/
├── pointcloud_map.pcd
├── lanelet2_map.osm
├── map_projector_info.yaml
└── map_config.yaml
```

角色分工要彻底分清：

- `my_navigation2/maps/*.pgm`：历史 2D 栅格地图，仅供 Nav2 或参考
- `my_cartographer`：2D/3D 前期勘察或建图实验工具
- `LIO-SAM-ROS2`：正式 `pointcloud_map.pcd` 生产工具链
- `Autoware map bundle`：正式实车运行地图资产

如果还没有正式点云地图，临时过渡可以用：

```bash
ros2 run wvcsc_autoware_bringup pgm_to_fake_pcd.py   --yaml /home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml   --output /home/robot/autoware_data/maps/wvcsc_fake_map/pointcloud_map.pcd
```

但要明确：

```text
fake pcd 只用于联调，不用于正式 NDT 定位交付
```

更完整的地图准备步骤见 [autoware_map_workflow.md](/home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_autoware_bringup/docs/autoware_map_workflow.md)。

## 7. `full_real_vehicle.launch.xml` 的逐层部署顺序

### 7.1 第一步：只拉硬件链

```bash
ros2 launch wvcsc_autoware_bringup hardware.launch.xml
```

这一层应该拉起：

- `wvcsc_vehicle_launch/vehicle.launch.xml`
- `wvcsc_sensor_kit_launch/sensing.launch.xml`
- `can_bridge`
- `wtb_car_driver`
- `wvcsc_vehicle_interface`
- `gnss_bridge.launch.xml`

重点检查 TF：

```text
base_footprint
base_link
sensor_kit_base_link
laser
gyro_link
gnss_link
```

重点检查 topic：

```text
/sensing/lidar/pointcloud_raw
/sensing/imu/tamagawa/imu_raw
/sensing/gnss/nav_sat_fix
/vehicle/status/velocity_status
/vehicle/status/steering_status
/car_odom
/twist_cmd
```

### 7.2 第二步：拉 localization

过渡 EKF：

```bash
ros2 launch wvcsc_autoware_bringup localization.launch.xml backend:=ekf
```

正式方向：

```bash
ros2 launch wvcsc_autoware_bringup localization.launch.xml backend:=autoware_ndt
```

当前建议先用 RViz 手动初始化位姿，因为：

- GNSS fix 已经 bridge 到 `/sensing/gnss/nav_sat_fix`
- 但 `/sensing/gnss/pose_with_covariance` 还未补齐

此阶段重点检查：

```text
/sensing/imu/imu_data
/localization/pose_estimator/pose_with_covariance
/localization/twist_estimator/twist_with_covariance
/localization/kinematic_state
```

### 7.3 第三步：只拉 Autoware 主栈

```bash
ros2 launch wvcsc_autoware_bringup autoware_stack.launch.xml   map_path:=/home/robot/autoware_data/maps/wvcsc_map   launch_perception:=false
```

这一层会打开：

- `map`
- `system`
- `planning`
- `control`
- `api`
- `rviz`

重点检查：

```text
/planning/mission_planning/route
/planning/trajectory
/control/command/control_cmd
/control/command/gear_cmd
/api/routing/state
/api/operation_mode/state
```

### 7.4 第四步：半实车模式

```bash
ros2 launch wvcsc_autoware_bringup hybrid_real_vehicle.launch.xml   map_path:=/home/robot/autoware_data/maps/wvcsc_map   localization_backend:=autoware_ndt
```

这个模式适合先验证：

```text
route -> planning -> control_cmd -> wvcsc_vehicle_interface -> /twist_cmd -> wtb_car_driver
```

建议：

- 低速 `0.1 ~ 0.2 m/s`
- 先短距离路线
- 保留人工接管

### 7.5 第五步：全实车模式

```bash
ros2 launch wvcsc_autoware_bringup full_real_vehicle.launch.xml   map_path:=/home/robot/autoware_data/maps/wvcsc_map   localization_backend:=autoware_ndt
```

当前它已经包含正式主链：

- `map`
- `perception`
- `planning`
- `control`
- `api`
- `system`

因此它已经具备：

```text
地图 -> 定位 -> 感知 -> 规划 -> 控制 -> 车辆接口 -> 底盘
```

但它和 `planning_simulator.launch.xml` 最大区别是：

- 没有 dummy vehicle
- 没有 dummy localization
- 没有 dummy perception

所以真链路任何一层缺口都会直接暴露出来。

## 8. 当前 planning/control 是否已经完整

答案是：

```text
是，当前 full_real_vehicle.launch.xml 已经通过 autoware.launch.xml 拉起正式 planning/control 主链。
```

其中规划层会下钻到：

- `mission_planner`
- `behavior_path_planner`
- `behavior_velocity_planner`
- `motion_velocity_planner`
- `scenario_selector`
- `velocity_smoother`

控制层会下钻到：

- `trajectory_follower_node`
- `vehicle_cmd_gate`
- `autoware_shift_decider`
- `operation_mode_transition_manager`
- `control_validator`
- `lane_departure_checker`
- `autonomous_emergency_braking`

控制输出链路是：

```text
/control/command/control_cmd
-> wvcsc_vehicle_interface
-> /twist_cmd
-> wtb_car_driver
```

所以当前真正的重点不是再补一个规划层或控制层入口，而是：

- 补正式地图资产
- 补定位初始化闭环
- 保证传感器输入质量

## 9. 推荐的完整部署顺序

建议严格按这个顺序推进：

1. 安装 `ros-humble-gtsam`，确保 `lio_sam` 能独立编译。
2. 检查 `/sensing/lidar/pointcloud_raw` 是否包含 `ring` 和 `time`。
3. 检查 `/sensing/imu/tamagawa/imu_raw` 频率、frame、时间同步。
4. 用 `run_wvcsc_mapping.launch.py` 做直播建图或 `run_wvcsc_offline_mapping.launch.py` 做离线建图。
5. 保存 `pointcloud_map.pcd` 并人工检查地图质量。
6. 补齐 `lanelet2_map.osm`、`map_projector_info.yaml`、`map_config.yaml`。
7. 先跑 `hybrid_real_vehicle.launch.xml`。
8. 再跑 `full_real_vehicle.launch.xml`。
9. 先 RViz 手动初始化，GNSS 自动初始化留到后续阶段。

## 10. 常见故障表

### 10.1 `GTSAM` 缺失

现象：

```text
Could not find a package configuration file provided by "GTSAM"
```

处理：

```bash
sudo apt-get install -y ros-humble-gtsam
```

### 10.2 LIO-SAM 一启动就抖动、扭曲、抽搐

优先检查：

- LiDAR 与 IMU 时间不同步
- 点云 `time` 字段格式不对
- IMU 外参错误
- `extrinsicTrans/extrinsicRot/extrinsicRPY` 不匹配

### 10.3 点云无法被 LIO-SAM 正常消费

优先检查：

- 是否有 `ring`
- 是否有 `time`
- 点类型是否被中间节点改写
- `/sensing/lidar/pointcloud_raw` 是否仍然是带多字段的机械雷达点云

### 10.4 NDT 有地图但定位不稳

优先检查：

- `pointcloud_map.pcd` 是否质量足够
- 是否还在用 fake pcd
- 初始位姿是否给对
- `map_projector_info.yaml` 是否合理
- `laser`、`base_link`、`gyro_link` TF 是否一致

### 10.5 `full_real_vehicle.launch.xml` 启动了但车不动

优先顺序：

1. 看 `/planning/trajectory` 是否存在。
2. 看 `/control/command/control_cmd` 是否存在。
3. 看 `wvcsc_vehicle_interface` 是否把控制命令转成 `/twist_cmd`。
4. 看 `wtb_car_driver` 是否接收到底盘指令。
5. 看底盘状态 `/vehicle/status/*` 是否正常回传。

## 11. 当前建议保留和不保留的文档边界

当前 `wvcsc_autoware_bringup/docs` 建议只长期保留：

- `real_vehicle_bringup.md`
- `autoware_map_workflow.md`
- `map_projector_info.example.yaml`
- `map_config.example.yaml`

其余与主手册内容重叠的 md 已建议合并收敛，避免后续越看越乱。

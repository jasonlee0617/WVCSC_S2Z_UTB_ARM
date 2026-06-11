# WVCSC `autoware.launch.xml` 实车数据流

## 1. 正式启动命令

以后 WVCSC 实车统一使用官方入口启动：

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/eisa/autoware_map/maps/wvcsc_map1 \
  vehicle_model:=wvcsc_vehicle \
  sensor_model:=wvcsc_sensor_kit \
  data_path:=/home/eisa/autoware_data
```

运行前提：

- `source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash` 会自动补齐：
  - `ROS_DOMAIN_ID=88`
  - `LD_LIBRARY_PATH` 中的 `~/.local/acados/lib`
- 这样可以避免 ROS 2 默认 domain 串网，也能保证 `path_optimizer` 的 `acados` 依赖可加载。

## 2. 启动层次

```text
autoware.launch.xml
├── global_params.launch.py
├── pointcloud_container.launch.py
├── tier4_vehicle_component
├── tier4_system_component
├── tier4_map_component
├── tier4_sensing_component
├── tier4_localization_component
├── tier4_perception_component
├── tier4_planning_component
├── tier4_control_component
├── tier4_autoware_api_component
└── rviz2
```

WVCSC 现在不再额外包一层 `wvcsc.launch.xml` 主入口，而是直接让官方 `autoware.launch.xml` 识别：

- `vehicle_model:=wvcsc_vehicle`
- `sensor_model:=wvcsc_sensor_kit`

## 3. 主数据流

### 3.1 传感器层

```text
雷神 C16 LiDAR
  -> lslidar_driver_node
  -> /sensing/lidar/pointcloud_raw
  -> pointcloud_xyzirt_to_xyzirc
  -> /sensing/lidar/concatenated/pointcloud

FDILink IMU
  -> ahrs_driver_node
  -> /imu
  -> relay
  -> /sensing/imu/imu_data

车辆速度反馈
  /vehicle/status/velocity_status
  -> autoware_vehicle_velocity_converter
  -> /sensing/vehicle_velocity_converter/twist_with_covariance
```

修复后的关键点：

- IMU `frame_id` 现在使用 `base_link`
- IMU 已补充非零协方差
- `wtb_car` 发布的 `/car_odom` 也已补充非零协方差

### 3.2 定位层

```text
/sensing/lidar/concatenated/pointcloud
  -> crop_box_filter_measurement_range
  -> voxel_grid_downsample_filter
  -> random_downsample_filter
  -> /localization/util/downsample/pointcloud

/localization/util/downsample/pointcloud
  + /map/pointcloud_map
  -> autoware_ndt_scan_matcher
  -> /localization/pose_estimator/pose_with_covariance

/localization/pose_estimator/pose_with_covariance
  + /localization/twist_estimator/twist_with_covariance
  -> autoware_ekf_localizer
  -> /localization/kinematic_state
  -> /localization/pose_twist_fusion_filter/pose
  -> TF: map -> base_link
```

### 3.3 规划层

```text
/map/vector_map + /localization/kinematic_state
  -> mission_planner
  -> /planning/mission_planning/route

route + pose + perception
  -> behavior_path_planner
  -> behavior_velocity_planner
  -> motion_velocity_planner
  -> velocity_smoother
  -> /planning/scenario_planning/trajectory
  -> trajectory_relay
  -> /planning/trajectory
```

### 3.4 控制层

```text
/planning/trajectory + /localization/kinematic_state
  -> trajectory_follower
  -> /control/trajectory_follower/control_cmd
  -> vehicle_cmd_gate
  -> /control/command/control_cmd
```

### 3.5 WVCSC 底盘适配层

```text
/control/control_mode_request
  -> wvcsc_vehicle_interface
  -> /vehicle/status/control_mode

/control/command/control_cmd
  -> wvcsc_vehicle_interface
  -> /twist_cmd
  -> wtb_car
  -> CAN
  -> 底盘 ECU

底盘反馈
  -> can_bridge
  -> wtb_car
  -> /car_odom
  -> /wtb_car_message
  -> wvcsc_vehicle_interface
  -> /vehicle/status/velocity_status
  -> /vehicle/status/steering_status
  -> /vehicle/status/gear_status
```

## 4. 当前 WVCSC 关键适配点

### 4.1 LiDAR 点云格式适配

- 原始点云：`PointXYZIRT`
- Autoware NDT 期望：`PointXYZIRC`
- 适配节点：`pointcloud_xyzirt_to_xyzirc`

### 4.2 IMU 话题适配

- 驱动输出：`/imu`
- Autoware 输入：`/sensing/imu/imu_data`
- 适配方式：`topic_tools relay`

### 4.3 交通灯心跳占位

WVCSC 当前没有真实交通灯识别链，因此补了：

```text
traffic_light_heartbeat.py
  -> /perception/traffic_light_recognition/traffic_signals
```

作用：

- 避免系统监控长期把 `traffic_signals` 当成硬错误
- 降低 `perception` 相关假故障对自动驾驶可用性的干扰

### 4.4 控制模式接管

`wvcsc_vehicle_interface` 已补齐：

- `/control/control_mode_request` 服务
- `/vehicle/status/control_mode` 状态发布
- 只有进入 Autoware 控制模式后，`/control/command/control_cmd` 才会下发到底盘

## 5. 本次排障后系统行为变化

### 5.1 为什么 LLT 不再刷屏

之前：

- IMU 协方差为零或接近零
- `/car_odom` 协方差也过小
- EKF 在融合 pose/twist 时更容易出现数值退化
- 最终表现为 LLT 分解报错反复刷屏

现在：

- IMU 协方差已显式配置为非零
- 里程计协方差也补成非零
- EKF 数值条件改善
- LLT 刷屏问题消失

### 5.2 为什么 `Auto` 的关键不在 RViz

`Auto` 是否可点，核心取决于这几条链路是否同时正常：

- 定位是否完成初始化
- fail-safe 是否处于 `NORMAL`
- duplicated-node checker 是否正常
- 控制模式服务和状态链路是否完整

也就是说，`Auto` 按钮只是最终表象，不是根因。

### 5.3 为什么官方 `autoware.launch.xml` 现在可以直接作为主入口

因为本次已经把 WVCSC 运行所需的关键差异前移到了更底层：

- `wvcsc_vehicle` / `wvcsc_sensor_kit` 模型本身可被官方入口识别
- 运行时 env hook 已自动设置 `ROS_DOMAIN_ID=88` 和 `acados` 路径
- 控制模式服务补齐
- LiDAR / IMU / 底盘协方差与状态链修复
- 交通灯占位心跳补齐

因此现在直接使用官方启动命令即可正常进入地图、定位、规划、控制流程。

## 6. 常看话题

```text
/sensing/lidar/pointcloud_raw
/sensing/lidar/concatenated/pointcloud
/sensing/imu/imu_data
/car_odom
/localization/kinematic_state
/planning/mission_planning/route
/planning/trajectory
/control/trajectory_follower/control_cmd
/control/command/control_cmd
/vehicle/status/velocity_status
/vehicle/status/control_mode
/system/fail_safe/mrm_state
```

## 7. 相关文档

- [`Autoware启动排障与修复复盘.md`](/home/eisa/WVCSC_S2Z_UTB_ARM/src/docs/Autoware启动排障与修复复盘.md)
- [`WVCSC_S2Z_UTB_ARM实车Autoware部署.md`](/home/eisa/WVCSC_S2Z_UTB_ARM/src/docs/WVCSC_S2Z_UTB_ARM实车Autoware部署.md)
- [`autoware_map_workflow.md`](/home/eisa/WVCSC_S2Z_UTB_ARM/src/docs/autoware_map_workflow.md)

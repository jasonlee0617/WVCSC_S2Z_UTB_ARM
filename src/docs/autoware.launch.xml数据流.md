# WVCSC Autoware 实车运行数据流

## 启动命令

```bash
source ~/autoware/install/setup.bash
source install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/eisa/autoware_map/maps/wvcsc_map \
  vehicle_model:=wvcsc_vehicle \
  sensor_model:=wvcsc_sensor_kit \
  data_path:=/home/eisa/autoware_data
```

## 启动层次

```
autoware.launch.xml
├── global_params.launch.py               (1) 全局参数
├── pointcloud_container.launch.py        (2) 点云处理容器
├── tier4_vehicle_component               (3) 车辆模块
├── tier4_system_component                (4) 系统监控
├── tier4_map_component                   (5) 地图加载
├── tier4_sensing_component               (6) 传感器驱动
├── tier4_localization_component          (7) 定位
├── tier4_perception_component            (8) 感知
├── tier4_planning_component              (9) 规划
├── tier4_control_component               (A) 控制
├── tier4_autoware_api_component          (B) API
└── RViz2                                 (C) 可视化
```

## 完整 Topic 数据流图

```
═══════════════════════════════════════════════════════════════════════
                          传 感 器 层
═══════════════════════════════════════════════════════════════════════

  雷神C16 LiDAR                       FDILink IMU
  UDP 192.168.1.200:2368              /dev/FDI_IMU_GNSS (串口)
       │                                    │
       ▼                                    ▼
  lslidar_driver_node                ahrs_driver_node
  ns: /sensing/lidar/cx              ns: /sensing
  frame: laser                       frame: gyro_link
       │                                    │
       │ /sensing/lidar/pointcloud_raw      │ /sensing/imu/tamagawa/imu_raw
       │ (PointXYZIRT, ~10Hz)               │ (sensor_msgs/Imu, ~100Hz)
       │                                    │
       ▼                                    ├──────────────────┐
  pointcloud_xyzirt_to_xyzirc              │                  │
  (XYZIRT → XYZIRC 转换)                   ▼                  ▼
       │                            relay              autoware_gyro_
       │ /sensing/lidar/            /sensing/imu/      odometer
       │ concatenated/pointcloud    tamagawa/imu_raw   │
       │ (PointXYZIRC, ~10Hz)       → /sensing/imu/    │ /localization/
       │                            imu_data           │ twist_estimator/
       │                            (~100Hz)           │ twist_with_covariance
       │                                               │ (~100Hz)
       ▼                                               │
  ┌────────────────────────────────────────────────────┘
  │
  ▼
  occupany_grid_map (感知用)     crop_box_filter_measurement_range
  /perception/occupancy_grid_    (定位用, ns:/localization/util)
  map/map                        /localization/util/
       │                         measurement_range/pointcloud
       │                              │
       ▼                              ▼
                                 voxel_grid_downsample_filter
                                 /localization/util/
                                 voxel_grid_downsample/pointcloud
                                      │
                                      ▼
                                 random_downsample_filter
                                 /localization/util/
                                 downsample/pointcloud (~10Hz, ≤3000点)


═══════════════════════════════════════════════════════════════════════
                          定 位 层
═══════════════════════════════════════════════════════════════════════

  /localization/util/downsample/pointcloud     /map/pointcloud_map
  (降采样后点云)                               (预建LIO-SAM地图)
       │                                             │
       └──────────────┬──────────────────────────────┘
                      ▼
              autoware_ndt_scan_matcher
              ns: /localization/pose_estimator
                      │
                      │ /localization/pose_estimator/pose_with_covariance (~10Hz)
                      │ /localization/pose_estimator/pose
                      ▼
              autoware_ekf_localizer
              ns: /localization/pose_twist_fusion_filter
              输入: NDT pose + gyro_odometer twist
                      │
                      │ /localization/kinematic_state (~50Hz)
                      │ /localization/pose_twist_fusion_filter/pose
                      │ TF: map → base_link
                      ▼
              autoware_stop_filter
              ns: /localization/pose_twist_fusion_filter
                      │
                      │ (过滤停止状态的速度抖动)


═══════════════════════════════════════════════════════════════════════
                          感 知 层
═══════════════════════════════════════════════════════════════════════

  /sensing/lidar/concatenated/pointcloud
       │
       ├──→ pointcloud_preprocessor (crop_box + downsample)
       │         │
       │         ▼
       │    lidar_centerpoint (3D检测)
       │    ns: /perception/object_recognition/detection
       │         │
       │         │ /perception/object_recognition/detection/centerpoint/objects
       │         ▼
       │    object_merger / tracker / predictor
       │         │
       │         │ /perception/object_recognition/objects (~10Hz)
       │         │ /perception/object_recognition/tracked_objects
       │
       └──→ traffic_light_classifier (需要摄像头, WVCSC当前无)
                │
                │ /perception/traffic_light_recognition/traffic_signals


═══════════════════════════════════════════════════════════════════════
                          规 划 层
═══════════════════════════════════════════════════════════════════════

  /map/vector_map (lanelet2)          /localization/kinematic_state
       │                                    │
       └────────────┬───────────────────────┘
                    ▼
            mission_planner + route_selector
            ns: /planning/mission_planning
                    │
                    │ /planning/mission_planning/route
                    ▼
            behavior_path_planner (行为规划)
            ns: /planning/scenario_planning/lane_driving/behavior_planning
            输入: route + 自车位姿 + 感知对象
                    │
                    │ /planning/scenario_planning/lane_driving/behavior_planning/path
                    ▼
            behavior_velocity_planner (速度规划)
            ns: /planning/scenario_planning/lane_driving/behavior_planning
                    │
                    │ /planning/scenario_planning/lane_driving/behavior_planning/trajectory
                    ▼
            motion_velocity_planner + velocity_smoother
            ns: /planning/scenario_planning/lane_driving/motion_planning
                    │
                    │ /planning/scenario_planning/trajectory (~10Hz)
                    ▼
            trajectory_relay
            /planning/scenario_planning/trajectory → /planning/trajectory


═══════════════════════════════════════════════════════════════════════
                          控 制 层
═══════════════════════════════════════════════════════════════════════

  /planning/trajectory          /localization/kinematic_state
       │                              │
       └──────────┬───────────────────┘
                  ▼
          trajectory_follower (controller_node_exe)
          ns: /control/trajectory_follower
          Pure Pursuit / MPC
                  │
                  │ /control/trajectory_follower/control_cmd
                  ▼
          vehicle_cmd_gate (命令门控: 紧急停车/超限保护)
          ns: /control
                  │
                  │ /control/command/control_cmd
                  │ (autoware_control_msgs/Control)
                  ▼


═══════════════════════════════════════════════════════════════════════
                          车 辆 层 (WVCSC 定制)
═══════════════════════════════════════════════════════════════════════

  /control/command/control_cmd          /control/command/gear_cmd
  (加速度 + 转向角)                     (档位: DRIVE/PARK/NEUTRAL)
       │                                      │
       └──────────────┬───────────────────────┘
                      ▼
              wvcsc_vehicle_interface
              (Autoware ↔ WTB 底盘适配器)
                      │
                      │ /twist_cmd (geometry_msgs/TwistStamped)
                      │   linear.x  = 速度 (m/s)
                      │   angular.z = 转向角 (rad)
                      ▼
              wtb_car (底盘驱动)
              Ackermann 运动学 → CAN 协议编码
                      │
                      │ CAN ID 0x18C4D2D0 (控制命令)
                      │ CAN ID 0x18C4D2EF (底盘反馈)
                      ▼
              can_bridge (USB-CAN 适配器)
              └─→ /can_rx (ROS2 → CAN)
              ┌─→ /can_tx (CAN → ROS2)
                      │
                      ▼
              ┌───────────────┐
              │   物理底盘     │
              │  电机 + 舵机   │
              └───────────────┘
                      │
                      ▼ (反馈)
              can_bridge → wtb_car
                      │
                      ├── /car_odom (nav_msgs/Odometry)
                      │     child_frame: base_footprint
                      │     用于 EKF 传感器融合
                      │
                      └── wvcsc_vehicle_interface
                            │
                            │ /vehicle/status/velocity_status
                            │ /vehicle/status/steering_status
                            │ /vehicle/status/control_mode
                            │ /vehicle/status/gear_status
                            ▼
                          Autoware (控制反馈)


═══════════════════════════════════════════════════════════════════════
                          TF 坐标变换树
═══════════════════════════════════════════════════════════════════════

  map
   │ (NDT + EKF, ~50Hz 动态)
   ▼
  base_link
   │
   ├── base_footprint          (static, z=-0.66, 地面投影)
   │
   ├── sensor_kit_base_link    (static, 零偏移)
   │    │
   │    ├── laser              (static, x=0.36, z=0.50)
   │    │    LiDAR 点云 frame
   │    │
   │    └── gyro_link          (static, z=-0.40)
   │         IMU 数据 frame
   │
   ├── left_front_link         (continuous joint, 前左轮)
   ├── right_front_link        (continuous joint, 前右轮)
   ├── left_wheel_link         (continuous joint, 后左轮)
   └── right_wheel_link        (continuous joint, 后右轮)

  map
   │ (vector_map_tf_generator, static)
   ▼
  viewer                      (地图可视化参考帧)


═══════════════════════════════════════════════════════════════════════
                      WVCSC 关键定制适配点
═══════════════════════════════════════════════════════════════════════

  1. pointcloud_xyzirt_to_xyzirc
     雷神C16输出 XYZIRT → Autoware NDT 期望的 XYZIRC
     /sensing/lidar/pointcloud_raw → /sensing/lidar/concatenated/pointcloud

  2. IMU relay
     /sensing/imu/tamagawa/imu_raw → /sensing/imu/imu_data
     fdilink_ahrs 发布的话题 → Autoware 期望的话题

  3. wvcsc_vehicle_interface
     Autoware control_cmd (加速度+转向角) → twist_cmd (速度+角速度)
     底盘 car_odom → Autoware velocity_status

  4. can_bridge + wtb_car
     USB-CAN 适配器 → CAN 协议编解码 → Ackermann 里程计

  5. lslidar_cx.yaml 三命名空间配置
     /cx, /lidar/cx, /sensing/lidar/cx — 适配不同 launch 拓扑


═══════════════════════════════════════════════════════════════════════
                        QoS 兼容性说明
═══════════════════════════════════════════════════════════════════════

  RELIABLE publisher (适配器) → BEST_EFFORT subscriber (crop_box_filter)
  在 ROS 2 中兼容 (订阅者可降级发布者的可靠性)

  LiDAR 点云:     深度 100, RELIABLE
  格式适配器:     深度 10,  RELIABLE
  NDT 预处理:     深度 5,   BEST_EFFORT
  IMU:            深度 2000, BEST_EFFORT


═══════════════════════════════════════════════════════════════════════
              车 辆 模 型 详 细 数 据 流
═══════════════════════════════════════════════════════════════════════

## 车辆模块启动链 (tier4_vehicle_component 内部)

```
tier4_vehicle_component.launch.xml
│
└── tier4_vehicle_launch/launch/vehicle.launch.xml
    │
    ├── (1) robot_state_publisher
    │    使用 tier4_vehicle_launch/urdf/vehicle.xacro 组合器:
    │    ┌─────────────────────────────────────────────┐
    │    │ tier4_vehicle_launch/urdf/vehicle.xacro    │
    │    │   ├── $(find wvcsc_vehicle_description)    │
    │    │   │   /urdf/vehicle.xacro                  │
    │    │   │   └── wvcsc_vehicle_base.xacro         │
    │    │   │       ├── base_footprint link          │
    │    │   │       ├── base_link link (1.30×0.80×1.00)│
    │    │   │       ├── base_link → base_footprint   │
    │    │   │       │   joint (fixed, z=-0.66)       │
    │    │   │       └── 4 wheels (continuous joint)  │
    │    │   │                                          │
    │    │   └── $(find wvcsc_sensor_kit_description) │
    │    │       /urdf/sensors.xacro                  │
    │    │       └── sensor_kit.xacro                 │
    │    │           ├── sensor_kit_base_link link    │
    │    │           ├── base_link → sensor_kit_      │
    │    │           │   base_link (fixed, 零偏移)     │
    │    │           ├── sensor_kit_base_link         │
    │    │           │   → laser (fixed)               │
    │    │           │   x=0.36, y=0, z=0.50          │
    │    │           └── sensor_kit_base_link         │
    │    │               → gyro_link (fixed)           │
    │    │               x=0, y=0, z=-0.40            │
    │    └─────────────────────────────────────────────┘
    │     输出: /robot_description (topic, Transient Local)
    │            /tf_static (fixed joints)
    │
    ├── (2) joint_state_publisher  ← WVCSC 添加
    │     wvcsc_vehicle_launch/vehicle_interface.launch.xml
    │     输出: /joint_states (wheel joints 零状态)
    │     触发 robot_state_publisher 发布轮子 TF
    │
    └── (3) vehicle_interface.launch.xml
          wvcsc_vehicle_launch/launch/vehicle_interface.launch.xml
          │
          ├── [launch_vehicle_hardware:=true]
          │   ├── can_bridge.launch.py
          │   │   └── can_bridge_node
          │   │       USB-CAN 设备 → can_msgs/Frame
          │   │       发布: /can_tx_1, /can_tx_2 (CAN→ROS)
          │   │       订阅: /can_rx_1, /can_rx_2 (ROS→CAN)
          │   │
          │   └── wtb_car node
          │       WHEELBASE=0.82, vel_scale=1.0, steer_offset=0.0
          │       订阅: /twist_cmd (TwistStamped)
          │       订阅: /can_tx_1, /can_tx_2 (CAN反馈帧)
          │       发布: /car_odom (Odometry, child:base_footprint)
          │       发布: /wtb_car_message (CarMsg: speed/angle/battery)
          │       发布: /can_rx_1, /can_rx_2 (控制帧→CAN)
          │       订阅: /run_static (start/stop 控制)
          │
          └── wvcsc_vehicle_interface_node
              参数: max_speed=1.0, max_steer=0.611, timeout=0.5s
              订阅: /control/command/control_cmd (Control)
              订阅: /control/command/gear_cmd (GearCommand)
              订阅: /car_odom (Odometry)
              订阅: /wtb_car_message (CarMsg)
              发布: /twist_cmd (TwistStamped)
              发布: /vehicle/status/velocity_status (VelocityReport)
              发布: /vehicle/status/steering_status (SteeringReport)
              发布: /vehicle/status/control_mode (ControlModeReport)
              发布: /vehicle/status/gear_status (GearReport)
              发布: /run_static (String, start/stop)


## 底盘 CAN 数据流详细

```
Autoware 控制命令
  /control/command/control_cmd
    longitudinal.velocity    (m/s, 目标速度)
    lateral.steering_tire_angle (rad, 目标转向角)
         │
         ▼
  wvcsc_vehicle_interface::onControlCmd()
    限幅: speed∈[-max_speed, max_speed], steer∈[-max_steer, max_steer]
         │
         ▼
  publishChassisCommand(speed, steer)
    /twist_cmd (TwistStamped)
      twist.linear.x  = speed   (m/s)
      twist.angular.z = steer   (rad)
         │
         ▼
  wtb_car::TwistCmdVelCallback()
    Ackermann 运动学:
      v = twist.linear.x * vel_scale
      δ = twist.angular.z + steer_offset
      ω = v * tan(δ) / WHEELBASE
         │
         ▼
    CAN 协议编码
      ID: 0x18C4D2D0 (8字节)
      Byte0-1: 目标速度 (0.001 m/s/bit)
      Byte2-3: 目标转向角 (0.001 rad/bit)
      Byte4:   档位 (0x00=N, 0x01=D, 0x02=R, 0x03=P)
      Byte5-7: 保留
         │
         ▼
    can_bridge → /can_rx_1 → USB-CAN → 底盘 ECU

  ───────────────── 反 馈 方 向 ─────────────────

  底盘 ECU → USB-CAN → can_bridge → /can_tx_1
         │
         ▼
  wtb_car::can_tx_callback()
    解析 CAN ID 0x18C4D2EF (反馈帧)
      Byte0-1: 当前速度 (0.001 m/s/bit)
      Byte2-3: 当前转向角 (0.001 rad/bit)
      Byte4:   当前档位
      Byte5-6: 电池电压 (0.1V/bit)
         │
    解析 CAN ID 0x18C4E2EF (辅助反馈)
      Byte0-3: 累计里程
         │
         ▼
    Ackermann 里程计计算
      v = 当前速度
      δ = 当前转向角
      ω = v * tan(δ) / WHEELBASE
      dt = 两次 CAN 帧时间差
      Δx = v * cos(θ) * dt
      Δy = v * sin(θ) * dt
      Δθ = ω * dt
      积分 → x_, y_, theta_
         │
         ▼
  publishOdom(x, y, theta, v, ω)
    /car_odom (Odometry)
      header.frame_id = "odom"
      child_frame_id  = "base_footprint"
      pose.pose.position = (x, y, 0)
      twist.twist.linear.x = v
      twist.twist.angular.z = ω
         │
         ├──→ wvcsc_vehicle_interface::onOdom()
         │     发布 /vehicle/status/velocity_status
         │       longitudinal_velocity = v
         │       lateral_velocity = 0
         │       heading_rate = ω
         │
         └──→ EKF (可选) → /ekf_odom


═══════════════════════════════════════════════════════════════════════
              传 感 器 套 件 详 细 数 据 流
═══════════════════════════════════════════════════════════════════════

## 传感器启动链 (tier4_sensing_component 内部)

```
tier4_sensing_component.launch.xml
│
└── tier4_sensing_launch/launch/sensing.launch.xml
    ns: /sensing
    │
    └── wvcsc_sensor_kit_launch/launch/sensing.launch.xml
        │
        ├── lidar.launch.xml                            ns: /sensing/lidar
        │   │
        │   ├── lslidar_cx_launch.py
        │   │   └── lslidar_driver_node                 ns: /sensing/lidar/cx
        │   │       参数文件: lslidar_cx.yaml
        │   │         [/sensing/lidar/cx/lslidar_driver_node]
        │   │         packet_rate: 847, pcl_type: false
        │   │         horizontal_angle_resolution: 0.18
        │   │         frame_id: laser
        │   │         topic_name: /sensing/lidar/pointcloud_raw
        │   │         scan_num: 7
        │   │       发布: /sensing/lidar/pointcloud_raw
        │   │         (PointXYZIRT: x,y,z,intensity,ring,time)
        │   │         ~10Hz, 每帧约 31,000 点
        │   │
        │   ├── pointcloud_to_laserscan_node
        │   │   (仅在 launch_pointcloud_to_laserscan:=true)
        │   │   cloud_in: /sensing/lidar/pointcloud_raw
        │   │   scan: /sensing/lidar/scan
        │   │   target_frame: laser
        │   │   min_height: -0.75, max_height: 0.50
        │   │
        │   └── pointcloud_xyzirt_to_xyzirc  ← WVCSC 添加
        │       wvcsc_common_sensor_launch
        │       输入: /sensing/lidar/pointcloud_raw
        │       输出: /sensing/lidar/concatenated/pointcloud
        │       转换: ring→channel, time保留, intensity→uint8
        │       格式: PointXYZIRC (x,y,z,intensity,return_type,channel)
        │
        ├── imu.launch.xml                              ns: /sensing
        │   │
        │   ├── ahrs_driver_autoware.launch.py
        │   │   └── ahrs_driver_node                    ns: /sensing
        │   │       参数文件: autoware_ahrs_params.yaml
        │   │       串口: /dev/FDI_IMU_GNSS, 921600bps
        │   │       发布: /sensing/imu/tamagawa/imu_raw
        │   │         (sensor_msgs/Imu)
        │   │         frame_id: gyro_link
        │   │         ~100Hz
        │   │         包含: orientation, angular_velocity,
        │   │               linear_acceleration
        │   │         额外发布: /gps/fix, /euler_angles,
        │   │                   /magnetic, /NED_odometry,
        │   │                   /system_speed, /mag_pose_2d
        │   │
        │   └── relay (topic_tools)  ← WVCSC 添加
        │       输入: /sensing/imu/tamagawa/imu_raw
        │       输出: /sensing/imu/imu_data
        │       (Autoware gyro_odometer/EKF 期望的 topic)
        │
        └── vehicle_velocity_converter
            输入: /vehicle/status/velocity_status
            输出: /sensing/vehicle_velocity_converter/
                  twist_with_covariance


## LiDAR 点云格式转换详情

```
LSLiDAR C16 原始数据包 (MSOP UDP)
  │  每包包含多个 firing (每个 firing 16 通道)
  │  每通道: 距离 + 强度
  │
  ▼
lslidar_driver_node 内部解码
  │  distance → (x, y, z) 球坐标转直角坐标
  │  计算 per-point time (相对扫描结束的负偏移)
  │  分配 ring (通道号 0-15)
  │
  ▼
PointXYZIRT (PCL 格式)
  float x, y, z          ← 直角坐标
  float intensity         ← 反射强度 (0-255)
  uint16_t ring           ← 通道号 (0-15, C16)
  float time              ← 相对时间 (LSLiDAR: 负值, 相对扫描结束)
  │
  ▼
pointcloud_xyzirt_to_xyzirc_node (格式适配器)
  │  读取 XYZIRT 字段，重组为 XYZIRC
  │  ring → channel (8bit)
  │  time → 保留不变
  │  intensity → 转为 0-255 uint8
  │
  ▼
PointXYZIRC (Autoware NDT 兼容格式)
  float x, y, z          ← 直角坐标
  uint8_t intensity       ← 反射强度
  uint8_t return_type     ← 回波类型 (默认 0)
  uint8_t channel         ← 通道号
  float time              ← (保留, 但NDT不使用)
  │
  ├──→ /sensing/lidar/concatenated/pointcloud
  │    (NDT 定位使用)
  │
  └──→ occupany_grid_map 节点
       (感知使用, 可选)
```


## IMU 数据流详情

```
FDILink AHRS 硬件
  │ 串口 /dev/FDI_IMU_GNSS @ 921600bps
  │ 协议: 自定义二进制帧 (FDILINK 协议)
  │ 帧类型: IMU (0x10), AHRS (0x11),
  │         INSGPS (0x12), GEODETIC (0x13)
  │
  ▼
ahrs_driver_node (fdilink_ahrs)
  │ processLoop() 串口读取
  │ 按帧头/长度解析 → 校验 CRC
  │
  ├── IMU 帧 → imu_publisher()
  │   /sensing/imu/tamagawa/imu_raw (100Hz)
  │   frame_id: gyro_link
  │   orientation: 四元数 (来自 AHRS 帧)
  │   angular_velocity: 陀螺仪原始值
  │   linear_acceleration: 加速度计原始值
  │   注意: gravity 未移除 (约 9.8 m/s² 在 z 轴)
  │
  ├── AHRS 帧 → 姿态计算
  │   /euler_angles (100Hz)
  │
  ├── INSGPS 帧 → GPS/INS 融合
  │   /gps/fix (NavSatFix)
  │   /NED_odometry
  │
  └── GEODETIC 帧 → GPS 位置
      /gps/fix (备用)

  ──────────── relay 分发 ────────────

  /sensing/imu/tamagawa/imu_raw (100Hz)
         │
         ├──→ (relay) → /sensing/imu/imu_data
         │              │
         │              ├──→ autoware_gyro_odometer
         │              │   /localization/twist_estimator/
         │              │   twist_with_covariance (~100Hz)
         │              │
         │              └──→ autoware_ekf_localizer
         │                  (EKF 传感器融合)
         │
         └──→ (LIO-SAM 建图时)
              lio_sam_imuPreintegration
```

## 点云预处理管道 (定位用)

```
/sensing/lidar/concatenated/pointcloud (XYZIRC, ~10Hz, ~31k点)
         │
         ▼
  ┌─────────────────────────────────────────────┐
  │ pointcloud_container                        │
  │ ns: / (root)                                │
  │ composable nodes:                           │
  │                                             │
  │ [1] crop_box_filter_measurement_range       │
  │     ns: /localization/util                  │
  │     输入: /sensing/lidar/concatenated/      │
  │           pointcloud                        │
  │     参数: input_frame=base_link             │
  │           min_x=-60, max_x=60               │
  │           min_y=-60, max_y=60               │
  │           min_z=-30, max_z=50               │
  │     输出: measurement_range/pointcloud      │
  │                                             │
  │ [2] voxel_grid_downsample_filter            │
  │     参数: voxel_size_x/y/z = 1.0m           │
  │     输入: measurement_range/pointcloud      │
  │     输出: voxel_grid_downsample/pointcloud  │
  │                                             │
  │ [3] random_downsample_filter                │
  │     参数: sample_num = 3000                 │
  │     输入: voxel_grid_downsample/pointcloud  │
  │     输出: /localization/util/downsample/    │
  │           pointcloud                        │
  └─────────────────────────────────────────────┘
         │
         ▼
  /localization/util/downsample/pointcloud (~10Hz, ≤3000点)
         │
         ▼
  autoware_ndt_scan_matcher
    配准: 当前降采样点云 ←→ 预建 pointcloud_map.pcd
    输出: /localization/pose_estimator/pose_with_covariance


═══════════════════════════════════════════════════════════════════════
              完 整 启 动 时 序
═══════════════════════════════════════════════════════════════════════

  T+0s   autoware.launch.xml 启动
  T+1s   global_params 加载 (vehicle_info.param.yaml)
  T+2s   pointcloud_container 容器启动
  T+3s   robot_state_publisher 发布 /robot_description + /tf_static
  T+4s   joint_state_publisher 发布 /joint_states → 轮子TF出现
  T+5s   can_bridge + wtb_car 启动, /car_odom 开始发布
  T+6s   wvcsc_vehicle_interface 启动, /vehicle/status/* 开始发布
  T+7s   LiDAR 驱动启动, DIFOP 握手成功
  T+8s   IMU 驱动启动, /sensing/imu/* 开始发布(100Hz)
  T+9s   格式适配器启动, /sensing/lidar/concatenated/pointcloud 开始发布(~10Hz)
  T+10s  map 组件加载 pointcloud_map.pcd + lanelet2_map.osm
  T+12s  定位 composable nodes 加载到 pointcloud_container
  T+13s  NDT scan matcher 就绪 (等待 2D Pose Estimate)
  T+14s  EKF localizer 就绪 (等待 NDT 初始化)
  T+15s  感知组件加载 (lidar_centerpoint 模型/TensorRT 转换)
  T+20s  规划组件就绪 (等待定位 + 地图 + 路线)
  T+25s  控制组件就绪 (等待规划轨迹)
  T+30s  系统完全就绪, RViz 可操作 2D Pose Estimate


═══════════════════════════════════════════════════════════════════════
              关 键 话 题 速 查 表
═══════════════════════════════════════════════════════════════════════

  ┌──────────────────────────────────┬──────────┬──────────────────────┐
  │ Topic                            │ 频率     │ 来源                 │
  ├──────────────────────────────────┼──────────┼──────────────────────┤
  │ /sensing/lidar/pointcloud_raw    │ ~10Hz    │ lslidar_driver       │
  │ /sensing/lidar/concatenated/     │ ~10Hz    │ pointcloud adapter   │
  │   pointcloud                     │          │                      │
  │ /sensing/imu/tamagawa/imu_raw    │ ~100Hz   │ fdilink_ahrs         │
  │ /sensing/imu/imu_data            │ ~100Hz   │ relay                │
  │ /sensing/vehicle_velocity_       │ ~10Hz    │ velocity_converter   │
  │   converter/twist_with_covariance│          │                      │
  │ /car_odom                        │ ~50Hz    │ wtb_car              │
  │ /twist_cmd                       │ on cmd   │ vehicle_interface    │
  │ /vehicle/status/velocity_status  │ ~10Hz    │ vehicle_interface    │
  │ /localization/util/downsample/   │ ~10Hz    │ random_downsample    │
  │   pointcloud                     │          │                      │
  │ /localization/pose_estimator/    │ ~10Hz    │ NDT scan matcher     │
  │   pose_with_covariance           │          │                      │
  │ /localization/kinematic_state    │ ~50Hz    │ EKF localizer        │
  │ /localization/twist_estimator/   │ ~100Hz   │ gyro_odometer        │
  │   twist_with_covariance          │          │                      │
  │ /control/command/control_cmd     │ ~10Hz    │ vehicle_cmd_gate     │
  │ /control/command/gear_cmd        │ on chg   │ gear interface       │
  │ /planning/trajectory             │ ~10Hz    │ trajectory relay     │
  │ /perception/object_recognition/  │ ~10Hz    │ object merger        │
  │   objects                        │          │                      │
  │ /map/pointcloud_map              │ once     │ map_loader           │
  │ /map/vector_map                  │ once     │ lanelet2_loader      │
  │ /robot_description               │ once     │ robot_state_publisher│
  │ /tf_static                       │ once     │ robot_state_publisher│
  │ /tf                              │ ~50Hz    │ EKF localizer        │
  └──────────────────────────────────┴──────────┴──────────────────────┘

# WVCSC Autoware 地图准备与转换指南

本文档只关注一件事：如何为 `full_real_vehicle.launch.xml` 准备正式可用的 Autoware 地图资产。

如果你想看整车 bringup、LIO-SAM 建图入口、逐层启动顺序，请看主手册 [real_vehicle_bringup.md](/home/eisa/WVCSC_S2Z_UTB_ARM/src/wvcsc_autoware_bringup/docs/real_vehicle_bringup.md)。

## 1. 先区分两类地图资产

### 1.1 `my_navigation2/maps/*.pgm + *.yaml`

这些文件是 Nav2 时代的 2D 栅格地图：

- 适合 AMCL
- 适合 Nav2 2D 占据栅格导航
- 适合表达墙体、障碍边界、空闲区域

它们不能直接当作 Autoware 正式地图的原因是：

- 没有 3D 几何细节
- 不能直接提供 NDT 所需的真实点云结构
- 不包含 Lanelet2 道路语义

### 1.2 Autoware 正式地图目录

建议最终统一准备成：

```text
/home/eisa/autoware_data/maps/wvcsc_map/
├── pointcloud_map.pcd
├── lanelet2_map.osm
├── map_projector_info.yaml
└── map_config.yaml
```

说明：

- `pointcloud_map.pcd`：定位用点云地图
- `lanelet2_map.osm`：任务级路线规划和道路语义地图
- `map_projector_info.yaml`：地图投影配置
- `map_config.yaml`：推荐保留的地图原点与姿态记录

## 2. 正式路线：使用 LIO-SAM 生成 `pointcloud_map.pcd`

当前 WVCSC 已经把 LIO-SAM 收敛成“建图优先”的独立工具链，推荐入口是：

- `ros2 launch lio_sam run_wvcsc_mapping.launch.py`
- `ros2 launch lio_sam run_wvcsc_offline_mapping.launch.py`

### 2.1 直播建图

```bash
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch lio_sam run_wvcsc_mapping.launch.py   launch_hardware:=true   launch_rviz:=true
```

直播建图时，`run_wvcsc_mapping.launch.py` 不直接重复发布车体 TF，而是通过 `launch_hardware:=true` 拉起 `hardware.launch.xml`：

- `map -> odom`：由 LIO-SAM 建图 launch 发布静态 TF
- `odom -> base_footprint`：由硬件链里的 EKF 发布，属于动态 `/tf`
- `base_footprint -> base_link -> sensor_kit_base_link -> laser/gyro_link`：由车体模型的 `robot_state_publisher` 发布，属于静态 `/tf_static`
- LIO-SAM：保持 `publishOdometryTf: false`，只输出建图结果 topic，避免抢占真实车体 TF
- GNSS：保持 `useGpsFactor: false`，GNSS topic/入口可以存在，但不参与当前室内 LIO-SAM 建图优化

如果 `launch_hardware:=false`，这条 launch 不会自动提供车体静态 TF 或 EKF 的 `odom -> base_footprint`。离线建图必须依赖 bag 中已有的 `/tf_static`，或单独启动车体模型与必要的里程 TF。

如果 RViz 里只看到 TF 坐标轴、看不到白色车体模型，优先检查 `RobotModel` 是否收到 `/robot_description`。这通常是模型显示或描述话题 QoS 问题，不是车体 TF 树断链。

建议建图前先确认：

```bash
ros2 topic hz /sensing/lidar/pointcloud_raw
ros2 topic hz /sensing/imu/tamagawa/imu_raw
ros2 topic echo /sensing/lidar/pointcloud_raw --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 topic echo --qos-durability transient_local /tf_static --once
ros2 topic echo /robot_description --once
```

重点确认：

- 点云中有 `ring`
- 点云中有 `time`
- LiDAR 与 IMU 时间同步
- `laser`、`gyro_link` 外参与实车一致
- `odom -> base_footprint` 的 `z` 接近 `0`
- `/tf_static` 中存在车体和传感器固定关节
- RViz 的 `RobotModel` 订阅 `/robot_description`，且 Durability 为 `Transient Local`

当前 WVCSC 建图链的正式 2D 扫描输出是 `/sensing/lidar/scan`，来自 `pointcloud_to_laserscan`。`/scan_raw` 只是雷达驱动在 `publish_scan=true` 时才会直接输出的兼容话题；在当前建图入口里默认关闭，不应作为 `run_wvcsc_mapping.launch.py` 的必需输入。

RViz 中不要只用 `/lio_sam/mapping/map_global` 判断建图是否稀疏。这个话题是全局地图可视化输出，会经过关键帧筛选和 voxel 降采样。建议同时观察：

- `/sensing/lidar/pointcloud_raw`：原始输入点云
- `/lio_sam/mapping/cloud_registered_raw`：当前高密度配准帧
- `/lio_sam/mapping/cloud_registered`：当前特征配准帧
- `/lio_sam/mapping/map_global`：全局可视化地图

LaserScan 辅助显示应使用 `/sensing/lidar/scan`，并在 RViz 中把 Reliability 设为 `Best Effort`。

### 2.2 离线建图

```bash
ros2 launch lio_sam run_wvcsc_offline_mapping.launch.py   launch_rviz:=false
```

再开一个终端播放 bag：

```bash
ros2 bag play <your_bag_path>
```

### 2.3 保存点云地图

先确认当前终端真的能识别 LIO-SAM 的服务接口，并且服务已经注册：

```bash
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 interface show lio_sam/srv/SaveMap
ros2 service list | grep save_map
ros2 service type /lio_sam/save_map
```

如果 `ros2 interface show lio_sam/srv/SaveMap` 失败，说明这个终端没有正确 source 当前工作区环境；这时直接执行 `ros2 service call` 会报 `The passed service type is invalid`。

确认无误后，再调用保存服务：

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '/home/eisa/autoware_data/maps/wvcsc_lio_sam'}"
```

建议保存后立刻检查：

```bash
ls -lh /home/eisa/autoware_data/maps/wvcsc_lio_sam
pcl_viewer /home/eisa/autoware_data/maps/wvcsc_lio_sam/GlobalMap.pcd
```

当前 WVCSC 版 LIO-SAM 已兼容两种保存路径写法：

- 绝对路径：`/home/eisa/autoware_data/maps/wvcsc_lio_sam`
- 相对 `HOME` 的路径：`/autoware_data/maps/wvcsc_lio_sam`

两种写法现在都会保存到同一个正确目录：

```text
/home/eisa/autoware_data/maps/wvcsc_lio_sam
```

如果你只启动了直播建图，不启动 `run_wvcsc_offline_mapping.launch.py`，完全可以直接保存地图；离线 launch 不是保存 `.pcd` 的前置条件。

地图应尽量避免：

- 墙体重影
- 转角撕裂
- 整体漂移
- 上下跳动

### 2.4 地图质量评估

如果 `pcl_viewer` 能正常打开 `GlobalMap.pcd`，说明直播建图和保存链路已经跑通。`vtkContextDevice2D` 这类 VTK 警告一般可以忽略，重点看 PCD 是否加载成功、点数是否合理、结构是否清楚。

当前短距离试扫图如果只有十几万点，例如 `153k` 左右，通常只能算“链路验证图”。它说明 LIO-SAM 已经能工作，但地图范围和密度还不适合直接作为正式 Autoware NDT 地图交付。

正式地图建议重点检查：

- `GlobalMap.pcd` 覆盖完整运行区域，不只是局部小块
- 墙体、立柱、边界没有明显双层重影
- 地面不要明显上下分层
- 转角和回到起点附近的位置不要撕裂
- 点云密度足够让 NDT 在主要区域稳定匹配
- 保存后的地图文件建议至少达到数 MB 级，点数应明显高于短距离试扫图

`pgm_to_fake_pcd.py` 生成的 fake map 不能用于评估 LIO-SAM 建图质量。fake map 只用于启动链验证，真实 `GlobalMap.pcd` 才用于评估点云定位质量。

### 2.5 直播建图采集技巧

WVCSC 当前使用 C16 雷达，点云天然比 32/64 线雷达稀疏。正式建图时要靠更慢、更完整、更重复的采集路线补密度。

推荐采集方式：

- 低速稳定行驶，建议 `0.1-0.3 m/s`
- 避免急加速、急刹、原地猛打方向
- 转弯尽量走大半径，减少 IMU/LiDAR 时间同步和外参误差被放大
- 从起点出发后绕一圈回到起点附近，给 LIO-SAM 闭环修正机会
- 同一区域建议正反各走一遍，第一遍建主体结构，第二遍补洞并检查重影
- 正式采集时尽量清场，避免人、车、移动设备留下拖影

保存前先确认这些话题持续更新：

```bash
ros2 topic hz /lio_sam/mapping/odometry
ros2 topic hz /lio_sam/mapping/path
ros2 topic hz /lio_sam/mapping/map_global
```

如果想保留更多细节，可以在地图稳定后尝试用更小保存分辨率：

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.1, destination: '/home/eisa/autoware_data/maps/wvcsc_lio_sam'}"
```

当前 `wvcsc_mapping_params.yaml` 已按室内 C16 雷达做第一轮温和加密：

```yaml
lidarMinRange: 0.3
odometrySurfLeafSize: 0.3
mappingSurfLeafSize: 0.3
globalMapVisualizationPoseDensity: 1.0
globalMapVisualizationLeafSize: 0.3
```

若后续地图仍明显稀疏，可以在确认 CPU 实时性足够后尝试把 `odometrySurfLeafSize`、`mappingSurfLeafSize`、`globalMapVisualizationLeafSize` 降到 `0.2`，再重新采集对比。如果出现卡顿、重影或轨迹不稳，则退回 `0.3`。

## 3. 正式路线：如何补齐另外三个地图文件

### 3.1 `lanelet2_map.osm`

即便已经有 `pointcloud_map.pcd`，Autoware 如果要做 mission planning 和 route planning，仍然需要 `lanelet2_map.osm`。

第一版最小要求：

- 可行驶 lanelet
- centerline
- 起终点连通关系
- 行驶方向

如果你的场地是封闭小车道路，建议先做“最小可用 lanelet2 图”，先把 route 跑通，再逐步细化。

### 3.2 `map_projector_info.yaml`

如果未来要接 GNSS，这个文件必须和地图坐标系一致。sample map 可参考：

```yaml
projector_type: MGRS
vertical_datum: WGS84
mgrs_grid: 54SVE
```

### 3.3 `map_config.yaml`

推荐记录：

- 地图原点纬度经度高程
- 地图姿态 `roll/pitch/yaw`

这样后续地图维护会轻松很多。

## 4. 临时联调路线：`pgm -> fake pcd`

这条路线只建议用于：

- 验证 launch 能不能跑通
- 验证 map 组件能不能加载
- 验证 localization / planning / control 能否串起来

不建议用于：

- 正式 NDT 定位
- 正式自动驾驶交付
- 地图质量评估

### 4.1 使用工具

```bash
ros2 run wvcsc_autoware_bringup pgm_to_fake_pcd.py   --yaml /home/eisa/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml   --output /home/eisa/autoware_data/maps/wvcsc_fake_map/pointcloud_map.pcd
```

如果图太密，可以抽样：

```bash
ros2 run wvcsc_autoware_bringup pgm_to_fake_pcd.py   --yaml /home/eisa/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml   --output /home/eisa/autoware_data/maps/wvcsc_fake_map/pointcloud_map.pcd   --sample-step 2
```

### 4.2 fake pcd 的边界

必须明确：

- 这是二维墙线挤压成的平面点云
- NDT 常常会不稳定
- 对初始位姿更敏感
- 长走廊和重复结构区域很容易错配

所以它只能作为：

```text
过渡联调工具
```

## 5. 推荐最终落地顺序

建议按下面顺序推进：

1. 用 LIO-SAM 采集并生成正式 `pointcloud_map.pcd`
2. 制作最小可用 `lanelet2_map.osm`
3. 补齐 `map_projector_info.yaml`
4. 补齐 `map_config.yaml`
5. 先用 `hybrid_real_vehicle.launch.xml` 联调
6. 再用 `full_real_vehicle.launch.xml` 做正式实车闭环

## 6. 和 `full_real_vehicle.launch.xml` 的关系

需要特别分清：

- `LIO-SAM` 负责地图生产
- `full_real_vehicle.launch.xml` 负责实车运行

当前推荐架构不是让 LIO-SAM 直接替代 Autoware 运行时定位，而是：

```text
LIO-SAM -> 生成 pointcloud_map.pcd
Autoware localization -> 消费 pointcloud_map.pcd 做运行时定位
```

这也是当前最适合 WVCSC 的路线。 

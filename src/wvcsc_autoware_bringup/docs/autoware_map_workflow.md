# WVCSC Autoware 地图准备与转换指南

本文档只关注一件事：如何为 `full_real_vehicle.launch.xml` 准备正式可用的 Autoware 地图资产。

如果你想看整车 bringup、LIO-SAM 建图入口、逐层启动顺序，请看主手册 [real_vehicle_bringup.md](/home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_autoware_bringup/docs/real_vehicle_bringup.md)。

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
/home/robot/autoware_data/maps/wvcsc_map/
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
source /home/robot/autoware/install/setup.bash
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch lio_sam run_wvcsc_mapping.launch.py   launch_hardware:=true   launch_rviz:=false
```

建议建图前先确认：

```bash
ros2 topic hz /sensing/lidar/pointcloud_raw
ros2 topic hz /sensing/imu/tamagawa/imu_raw
ros2 topic echo /sensing/lidar/pointcloud_raw --once
```

重点确认：

- 点云中有 `ring`
- 点云中有 `time`
- LiDAR 与 IMU 时间同步
- `laser`、`gyro_link` 外参与实车一致

### 2.2 离线建图

```bash
ros2 launch lio_sam run_wvcsc_offline_mapping.launch.py   launch_rviz:=false
```

再开一个终端播放 bag：

```bash
ros2 bag play <your_bag_path>
```

### 2.3 保存点云地图

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: '/home/robot/autoware_data/maps/wvcsc_lio_sam'}"
```

建议保存后立刻检查：

```bash
pcl_viewer /home/robot/autoware_data/maps/wvcsc_lio_sam/GlobalMap.pcd
```

地图应尽量避免：

- 墙体重影
- 转角撕裂
- 整体漂移
- 上下跳动

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
ros2 run wvcsc_autoware_bringup pgm_to_fake_pcd.py   --yaml /home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml   --output /home/robot/autoware_data/maps/wvcsc_fake_map/pointcloud_map.pcd
```

如果图太密，可以抽样：

```bash
ros2 run wvcsc_autoware_bringup pgm_to_fake_pcd.py   --yaml /home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml   --output /home/robot/autoware_data/maps/wvcsc_fake_map/pointcloud_map.pcd   --sample-step 2
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

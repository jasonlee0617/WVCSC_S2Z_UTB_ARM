# WVCSC Autoware 地图准备与转换

本文档只关注两件事：

1. 如何生产 `wvcsc_map1` 的正式地图资产  
2. 如何区分正式建图和临时 fake map 联调  

如果你想看正式实车部署，请看：

- [`WVCSC_S2Z_UTB_ARM实车Autoware部署.md`](/home/eisa/WVCSC_S2Z_UTB_ARM/src/docs/WVCSC_S2Z_UTB_ARM实车Autoware部署.md)

## 1. 正式地图目录

当前实车运行统一使用：

```text
/home/eisa/autoware_map/maps/wvcsc_map1
```

建议目录内容：

```text
pointcloud_map.pcd
lanelet2_map.osm
map_projector_info.yaml
map_config.yaml
```

## 2. 两类路线要分清

### 2.1 正式路线：LIO-SAM 建图

用途：

- 生产正式 `pointcloud_map.pcd`
- 用于 NDT 定位
- 用于最终实车部署

### 2.2 临时路线：`pgm -> fake pcd`

用途：

- 只做 launch 链路验证
- 只做 map 组件加载验证
- 不用于正式 NDT 定位

## 3. LIO-SAM 正式建图

### 3.1 直播建图

```bash
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch lio_sam run_wvcsc_mapping.launch.py \
  launch_hardware:=true \
  launch_rviz:=false
```

### 3.2 离线建图

```bash
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source /home/eisa/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch lio_sam run_wvcsc_offline_mapping.launch.py \
  launch_rviz:=false
```

另一个终端播放 bag：

```bash
ros2 bag play <your_bag_path>
```

### 3.3 建图前检查

```bash
ros2 topic hz /sensing/lidar/pointcloud_raw
ros2 topic hz /imu
ros2 topic hz /sensing/imu/imu_data
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link gyro_link
```

要点：

- 点云必须稳定
- IMU 必须稳定
- 外参与车体一致

## 4. 保存地图

先确认接口存在：

```bash
ros2 interface show lio_sam/srv/SaveMap
ros2 service list | grep save_map
```

再保存：

```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \
  "{resolution: 0.2, destination: '/home/eisa/autoware_map/maps/wvcsc_map1'}"
```

保存后检查：

```bash
ls -lh /home/eisa/autoware_map/maps/wvcsc_map1
```

## 5. 地图验收标准

正式 `pointcloud_map.pcd` 建议满足：

- 覆盖完整运行区域
- 墙体、路沿、立柱清晰
- 不明显双层重影
- 回到起点附近时不明显撕裂
- 点云密度足够让 NDT 稳定匹配

如果只是短距离试扫、小区域测试、点数很少，这通常只能算链路验证图。

## 6. `lanelet2_map.osm` 最小要求

即便 `pointcloud_map.pcd` 已经准备好，正式路线规划仍然需要 `lanelet2_map.osm`。

第一版最小要求：

- 可行驶 lanelet
- centerline
- 起终点连通关系
- 正确行驶方向

## 7. `map_projector_info.yaml`

如果地图需要和 GNSS 或投影坐标系配套，至少要保留：

```yaml
projector_type: MGRS
vertical_datum: WGS84
mgrs_grid: 54SVE
```

实际内容以你的地图坐标系为准。

## 8. `map_config.yaml`

推荐记录：

- 地图原点经纬度高程
- 地图姿态 `roll/pitch/yaw`
- 地图采集时间、版本、采集人

## 9. 临时联调：`pgm -> fake pcd`

只适用于：

- launch 能否跑通
- 地图能否被加载
- localization / planning / control 是否能串起来

不适用于：

- 正式交付
- 正式 NDT 定位
- 地图质量评估

### 9.1 生成命令

```bash
ros2 run my_navigation2 pgm_to_fake_pcd.py \
  --yaml /home/eisa/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml \
  --output /home/eisa/autoware_map/maps/wvcsc_map1/pointcloud_map.pcd
```

需要降采样时：

```bash
ros2 run my_navigation2 pgm_to_fake_pcd.py \
  --yaml /home/eisa/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml \
  --output /home/eisa/autoware_map/maps/wvcsc_map1/pointcloud_map.pcd \
  --sample-step 2
```

### 9.2 边界说明

fake pcd 本质上是二维栅格边界的平面挤压：

- 对初始位姿更敏感
- 长走廊、重复结构很容易错配
- 只能做过渡联调

## 10. 推荐落地顺序

1. LIO-SAM 采集并保存正式 `pointcloud_map.pcd`
2. 制作最小可用 `lanelet2_map.osm`
3. 补齐 `map_projector_info.yaml`
4. 补齐 `map_config.yaml`
5. 用正式地图目录 `/home/eisa/autoware_map/maps/wvcsc_map1` 做实车联调

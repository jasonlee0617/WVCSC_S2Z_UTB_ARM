# WVCSC_S2Z_UTB_ARM 实车 Autoware 部署

本文档描述当前推荐的正式实车部署方式：**直接使用官方 `autoware.launch.xml` 作为唯一主入口。**

## 1. 正式主入口

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

## 2. 硬件准备

上电前先确认：

- 工控机供电正常
- LiDAR 已接网线并上电
- IMU 已连接到 `/dev/FDI_IMU_GNSS`
- USB-CAN 已接入
- 底盘已上电

## 3. 工控机环境

### 3.1 基础环境

- Ubuntu 22.04
- ROS 2 Humble
- Autoware 工作区：`/home/eisa/autoware`
- WVCSC 工作区：`/home/eisa/WVCSC_S2Z_UTB_ARM`

### 3.2 source 顺序

必须按下面顺序：

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

第三步完成后，运行环境会自动带上：

- `ROS_DOMAIN_ID=88`
- `~/.local/acados/lib` 的动态库路径

可以手动确认：

```bash
echo $ROS_DOMAIN_ID
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep acados
```

## 4. 编译

### 4.1 编译 Autoware

见文档：

- [`Autoware.universe源码下载编译.md`](/home/eisa/WVCSC_S2Z_UTB_ARM/src/docs/Autoware.universe源码下载编译.md)

### 4.2 编译 WVCSC 工作区

```bash
cd /home/eisa/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. 地图目录准备

当前运行命令固定使用：

```text
/home/eisa/autoware_map/maps/wvcsc_map1
```

这个目录至少应包含：

```text
pointcloud_map.pcd
lanelet2_map.osm
map_projector_info.yaml
```

推荐再保留：

```text
map_config.yaml
```

如果还在准备地图，请看：

- [`autoware_map_workflow.md`](/home/eisa/WVCSC_S2Z_UTB_ARM/src/docs/autoware_map_workflow.md)

## 6. 运行前检查

### 6.1 LiDAR 网口/IP

```bash
ip addr
ping 192.168.1.200
ros2 topic hz /sensing/lidar/pointcloud_raw
```

### 6.2 IMU 串口

```bash
ls -l /dev/FDI_IMU_GNSS
ros2 topic hz /imu
ros2 topic hz /sensing/imu/imu_data
```

### 6.3 CAN / USB-CAN

```bash
ros2 topic echo /car_odom --once
ros2 topic echo /vehicle/status/velocity_status --once
```

### 6.4 地图路径

```bash
ls /home/eisa/autoware_map/maps/wvcsc_map1
```

### 6.5 ROS 2 domain

```bash
echo $ROS_DOMAIN_ID
```

预期：

```text
88
```

## 7. 启动命令

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

## 8. RViz 初始化流程

首次进入后建议按这个顺序操作：

1. 等待地图、点云、车体模型都加载出来
2. 选择 `2D Pose Estimate`
3. 在当前车辆真实位置附近给初始位姿
4. 等待定位稳定
5. 选择 `2D Goal`
6. 点击目标点

判断定位是否完成，可看：

```bash
ros2 topic echo /localization/initialization_state --once
ros2 topic echo /localization/kinematic_state --once
```

## 9. 路线规划与自动驾驶接管

### 9.1 现场操作顺序

推荐顺序：

1. `2D Pose Estimate`
2. `2D Goal`
3. 确认路线已生成
4. 确认 fail-safe 正常
5. 使能 Autoware 控制
6. 进入自动模式

### 9.2 关键检查项

#### 路线是否生成

```bash
ros2 topic echo /planning/mission_planning/route --once
ros2 topic echo /planning/route_state --once
```

#### fail-safe 是否正常

```bash
ros2 topic echo /system/fail_safe/mrm_state --once
```

正常期望：

- `state: 1`
- `behavior: 1`

#### 控制模式链路是否正常

```bash
ros2 topic echo /vehicle/status/control_mode --once
ros2 topic echo /system/operation_mode/state --once
```

#### 最终控制命令是否下发

```bash
ros2 topic echo /control/command/control_cmd --once
ros2 topic echo /twist_cmd --once
```

## 10. 常见告警与排查

### 10.1 `Auto` 不可用

优先检查：

- 是否已经做了 `2D Pose Estimate`
- `/system/fail_safe/mrm_state` 是否正常
- 是否存在 duplicated nodes
- `/vehicle/status/control_mode` 是否在变化

### 10.2 能规划但不走车

优先检查：

- `/planning/trajectory` 是否存在
- `/control/command/control_cmd` 是否有速度输出
- `/twist_cmd` 是否真正下发
- `vehicle_cmd_gate` 是否进入 `Emergency`

### 10.3 LiDAR 无点云

```bash
ping 192.168.1.200
ros2 topic hz /sensing/lidar/pointcloud_raw
```

### 10.4 IMU 无数据

```bash
ls -l /dev/FDI_IMU_GNSS
ros2 topic hz /imu
```

### 10.5 路径优化相关动态库错误

```bash
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep acados
```

## 11. 关于 `wvcsc_autoware_bringup`

当前结论：

- 它不再作为正式启动入口
- 运行时所需 env hook、硬件 launch、工具脚本、文档都在迁移
- 长期目标是删除这个包

现在的推荐结构是：

- `autoware_launch`：唯一正式入口
- `wvcsc_vehicle_launch`：车辆相关 launch 与 env hook
- `wvcsc_sensor_kit_launch`：传感器相关 launch 与占位心跳
- `my_navigation2`：过渡地图工具
- `src/docs/`：唯一对外文档目录

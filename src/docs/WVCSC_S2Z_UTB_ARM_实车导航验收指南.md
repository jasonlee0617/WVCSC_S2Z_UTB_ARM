# WVCSC 实车五点导航与喷洒验收指南

本文档描述当前实车主流程：建图/定位、采集五个导航点、自动执行五点路线、广域喷洒和机械臂定点喷洒。

当前实车任务唯一入口为：

```text
real_system_mission.launch.py
  → preflight
  → real_orchestration.launch.py
  → field_route_manager
  → 五点导航 + 第1路广域喷洒 + 第2路机械臂喷洒
```

`wvcsc_mission_manager` 和 `/mission/load_manual` 仅保留给仿真兼容；不再作为实车任务入口。

## 1. 前置条件

- 小车底盘、LiDAR、IMU、Alicia-M、C10 和继电器硬件已连接；
- 物理急停可用，首次测试使用低速；
- 已完成 C10 内参和手眼标定；
- 实车 YOLO 权重已放入 `wvcsc_rgb_vision/models/`；
- 继电器串口配置已确认。

先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

机械臂串口和继电器串口是独立设备：

```text
serial_port       → Alicia-M，默认 /dev/ttyACM0
fault.ini PortName → Modbus 继电器，默认 /dev/ttyUSB0
```

现场建议使用 `/dev/serial/by-id/` 的稳定设备名。

## 2. 建图

```bash
ros2 launch wvcsc_bringup real_cartographer.launch.py
```

低速遥控小车覆盖完整作业区域，确认地图无重影、无明显漂移后保存：

```bash
bash "$(ros2 pkg prefix wvcsc_bringup)/share/wvcsc_bringup/scripts/save_corn_map.sh"
```

地图文件示例：

```text
~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml
```

## 3. 启动定位并初始化 AMCL

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py
```

在 RViz 中使用 `2D Pose Estimate` 设置初始位姿，确认：

- 地图正常显示；
- `/amcl_pose` 持续发布；
- `/map → /base_footprint` TF 可用；
- `/imu` 和 `/ekf_odom` 有数据。

`real_navigation.launch.py` 只用于定位和导航诊断，不启动机械臂喷洒任务。

## 4. 创建并采集五点路线

复制模板：

```bash
ROUTE_FILE="$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/real/field_route_corn.yaml"
MAP_FILE="$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml"
cp "$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/real/field_route_corn.example.yaml" "$ROUTE_FILE"
```

小车通过 RViz `Navigation2 Goal` 或遥控器移动到每个点，车辆停止后依次执行：

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_1
```

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_2 \
  --tree-id corn_01 --tree-x-m 0.0 --tree-y-m 1.50
```

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_3 \
  --tree-id corn_02 --tree-x-m 0.0 --tree-y-m 1.50
```

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_4

ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_5
```

坐标约定：

```text
+X：车头方向
+Y：车体左侧
-Y：车体右侧
```

`point_2` 和 `point_3` 的树坐标必须使用机械臂基座坐标系下的带符号 X/Y。

### 4.1 采点门控说明

默认采点为宽松模式：

- 仍需要初始 `/imu`、`/ekf_odom`、`/amcl_pose` 和有效 `map → base_footprint` TF；
- 质量、协方差、样本散布、短暂数据过期和地图 footprint 不再阻止写入；
- 质量数据仍保存到 `capture_quality`，异常只输出 warning；
- 不要求使用 `--force-capture`。

最终工程验收时可显式启用严格门控：

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE" \
  --route-point point_1 --strict-capture --update
```

## 5. 验证路线文件

日常任务验证使用宽松模式：

```bash
ros2 run wvcsc_bringup validate_field_route.py -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE"
```

输出中应包含：

```text
[FIELD_ROUTE][VALID] ... steps=point_1,point_2,point_3,point_4,point_5
[FIELD_ROUTE][WARN] capture quality and footprint gates are disabled
```

最终验收可使用：

```bash
ros2 run wvcsc_bringup validate_field_route.py -- \
  --file "$ROUTE_FILE" --map "$MAP_FILE" --strict
```

基础结构、地图绑定、树距离、树 ID 和喷洒时长即使在宽松模式下仍然严格校验。

## 6. 启动完整五点实车任务

确认继电器服务配置：

```bash
ros2 service type /relay/set
```

应返回：

```text
wvcsc_interfaces/srv/SetRelay
```

启动任务：

```bash
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  mission_file:="$ROUTE_FILE" \
  map:="$MAP_FILE" \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python" \
  relay_config_file:="${HOME}/relay_fault.ini"
```

不传 `relay_config_file` 时，使用安装包中的 `controller_pkg/config/fault.ini`。

## 7. 五点任务行为

```text
point_1:
  第1路广域喷洒开启 → 导航

point_2:
  第1路关闭 → 车辆停稳 → 机械臂识别病害并通过第2路喷洒3秒
  → 第1路重新开启

point_3:
  第1路关闭 → 车辆停稳 → 机械臂识别病害并通过第2路喷洒3秒
  → 继续导航

point_4:
  第1路关闭 → 继续导航

point_5:
  第1、2路再次关闭 → 车辆停稳 → 任务完成
```

车辆到达 `point_2`、`point_3` 和 `point_5` 后，仍会执行停稳检查；该检查是运行安全门控，避免车辆运动时机械臂动作，不属于采点质量门控。

任意导航、继电器、机械臂、病害识别、取消或超时失败，都会：

1. 取消当前 Nav2/机械臂 Action；
2. 请求第 1、2 路继电器断开；
3. 发布 `FAILED` 状态并停止任务。

## 8. 运行监控

```bash
ros2 topic echo /mission/status
ros2 topic hz /camera/color/image_raw
ros2 topic hz /vision/tree_debug_image
ros2 topic hz /vision/diseased_target_debug_image
ros2 service type /relay/set
```

预期日志包括：

```text
[FIELD_ROUTE] services ready; auto-starting mission=...
[FIELD_ROUTE] arrived at point_1
[FIELD_ROUTE] arrived at point_2
[SPRAY] service mode, relay service=/relay/set channel=2
[FIELD_ROUTE] arrived at point_3
[FIELD_ROUTE][SUCCESS] five-point route completed
```

## 9. 取消与急停

任务取消：

```bash
ros2 service call /field_route/cancel std_srvs/srv/Trigger "{}"
```

机械臂停止/回 HOME：

```bash
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: reset}"
```

现场必须保留可触达的车辆物理急停。

## 10. 当前已移除的旧实机入口

以下入口不再安装或用于实车主流程：

- `load_site_mission.py`；
- `nav_validate_sites.py`；
- `/mission/load_manual` 作为实车任务启动接口；
- 旧 measured-site mission 逐树自动任务流程。

仿真仍保留 `wvcsc_mission_manager`，因此不要从工作区删除该 ROS 包。

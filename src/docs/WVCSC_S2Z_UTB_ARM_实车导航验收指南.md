# WVCSC 实车任意路线导航与喷洒验收指南

当前实车默认使用 **Qt 驱动的任意路线任务**。它不再限制为五个点，适用于当前
23 株病株以及任意数量的通行点、病株检查点和终点。保留原有 YAML 五点路线与
`capture_site_pose` 命令行采点，作为已验证的兼容模式和继电器联调工具。

默认数据流为：

```text
real_system_mission.launch.py (mission_mode:=qt)
  → Nav2 / AMCL / RViz + Qt 任务编辑器
  → 操作者确认初始位姿、编辑路线并点击“多点导航+喷洒”
  → mission_manager
  → Nav2 + 第1路广域喷洒 + 第2路机械臂喷洒
```

启动后不会自动行驶。只有 Qt 向 `/mission/load_manual` 提交路线并调用
`/mission/start` 后才开始执行；因此必须先在 RViz 完成 `2D Pose Estimate`。

## 1. Qt 任意路线作业（默认）

### 1.1 启动与初始定位

建图完成后，不要再单独启动 `real_navigation.launch.py`。完整任务入口已经启动同一套
底盘、LiDAR、IMU、EKF、AMCL、Nav2、RViz、C10、YOLO、MoveIt 和继电器；同时再启动
独立导航会造成 Nav2、AMCL、TF 和 `/navigate_to_pose` 冲突。

```bash
source /opt/ros/humble/setup.bash
source "$HOME/WVCSC_S2Z_UTB_ARM/install/setup.bash"

ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="$HOME/venvs/wvcsc_yolo_ros/bin/python"
```

实车默认硬件为 C10 `/dev/video2`、Alicia-M `/dev/ttyACM0`。设备号变化时只在本次
启动显式覆盖 `c10_device:=...` 或 `serial_port:=...`，不要改动 YOLO 节点代码。

在弹出的导航 RViz 中：

1. 选择 `2D Pose Estimate`，在地图上点击并拖动，设置小车当前位置和朝向；
2. 等待 AMCL 位姿与机器人模型稳定；
3. 再到 Qt 窗口点击“记录起点”。该起点只用于可选的任务完成后返回，不会触发导航。

### 1.2 通过 RViz 与 Qt 记录路线

Qt 的“点类型”就是人工确认点位属性的入口：

| 点类型 | 人工含义 | RViz 操作 | 到点动作 |
| --- | --- | --- | --- |
| 通行点 `TRANSIT` | 健康株附近或仅需经过的位置 | 一次 `2D Goal`，Qt 点击“使用最新目标为停靠位” | 可选停留；不驱动机械臂 |
| 病株检查点 `INSPECT` | 需要机械臂视觉识别/单独喷洒的玉米树 | 第一次 `2D Goal` 是停车位，点击“使用最新目标为停靠位”；第二次 `2D Goal` 点该树中心，再点击“使用下一目标为树中心” | 关闭第1路、停稳、机械臂识别并第2路喷洒 |
| 终点 `FINISH` | 路线结束位置 | 一次 `2D Goal`，Qt 点击“使用最新目标为停靠位” | 第1、2路最佳努力关闭，可选停留；不驱动机械臂 |

`2D Goal` 只提供地图平面位置和航向，不会自动识别“第几株玉米”或自动判断病害；
LiDAR 能看到行侧障碍，但当前工程没有可靠的单株玉米中心/ID提取器。因此病株中心仍由
操作者在 RViz 点击，Qt 才能计算机械臂所需的相对坐标。

对于每个 `INSPECT`，Qt 使用停车位与树中心的两个 map 坐标，并按实际安装关系
`alicia_base_link = base_footprint + (-0.40, 0, pi)` 计算带符号的
`tree_x_m/tree_y_m`。显示为：

```text
tree_y_m > +0.05 m  → 左侧预设
tree_y_m < -0.05 m  → 右侧预设
abs(tree_y_m) <= 0.05 m → 拒绝提交，需要重新选择停车位或树中心
```

这两个符号属于 `alicia_base_link`，不是车体坐标，也不应手动按“地图左/右”猜测。
左、右两侧均使用已人工提供的独立关节预设；不会用镜像角度替代。

### 1.3 广域喷洒、病株喷洒和顺序

“驶向该点时开启广域喷洒”是 **到该点的入段属性**，不是健康/病害标签。
例如路线为 `点1(通行) → 点2(病株) → 点3(病株) → 点4(通行) → 点5(终点)` 时：

```text
点1 incoming wide=开：车辆确认开始运动后，第1路开启
点2 incoming wide=开：到点后关闭第1路，车辆停稳，机械臂执行第2路喷洒
点3 incoming wide=开：点2完成且车辆再次起步后，第1路重新开启；到点重复病株流程
点4 incoming wide=开：点3完成后再次起步时第1路开启
点5 incoming wide=关：发送最终导航前关闭第1路；终点关闭两路
```

因此“在不到达病树前都执行广域喷洒”只在你为每一个入段勾选广域喷洒时成立；到达
病株停车位后广域喷洒一定关闭，避免机械臂运动时第1路仍工作。第1路只有在 Nav2 已
接受目标且 `/ekf_odom` 线速度达到 `0.03 m/s` 后才尝试开启；继电器通信失败当前只记
录告警，路线继续。

在表格内可修改/排序/删除各点，并设置每点的广域入段开关、病株喷洒时长、停留时间和
侧位。点击“保存多点任务”保存 JSON，点击“加载多点任务”恢复；勾选“完成后返回起点”
时，最后一个点完成后关闭第1路并导航回起点。

### 1.4 开始、跳过与终止

检查表格、路线标记和起点后点击“多点导航+喷洒”（单点路线可点击“单点导航+喷洒”）。
任务不会因为一个点的已确认 Nav2 失败、视觉失败、观察失败或继电器调用失败而整体中断：
该点会标记为跳过，继续后续点。仍会在导航/机械臂 Action 超时、取消尚未确认、运动锁定
或关键定位状态缺失时停止，避免与未结束动作并发。

“终止作业并回HOME”调用 `/mission/abort_and_home`：先取消导航和机械臂 Action，最佳
努力关闭第1、2路，再通过 `motion_control` 发送 `stop` 和 `reset`，等待其 `HOME_LOCKED`
或 `RESET_FAILED` 状态；它不是固定延时后直接下发复位命令。现场急停和物理断电仍是
继电器异常时的最后安全手段。

## 2. 前置条件

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
video_device      → C10 相机，默认 /dev/video2
fault.ini PortName → Modbus 继电器，默认 /dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.0-port0
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
~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/map_YYYYMMDD_HHMMSS/orchard.yaml
```

保存脚本会自动创建时间戳目录。导航启动时自动选择最新的
`map_YYYYMMDD_HHMMSS/orchard.yaml`，不需要手动填写 `map:=...`。

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

## 3. 兼容模式：YAML 五点路线与命令行采点

复制模板到新的时间戳任务目录：

```bash
MISSION_DIR="$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/real/mission_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$MISSION_DIR"
ROUTE_FILE="$MISSION_DIR/field_route_corn.yaml"
MAP_FILE="$(python3 -c 'from wvcsc_bringup.path_defaults import latest_map_yaml; print(latest_map_yaml())')"
cp "$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/real/field_route_corn.example.yaml" "$ROUTE_FILE"
```

模板中的 `REPLACE_BY_CAPTURE_TOOL` 会在第一次采点时自动绑定当前
`--map` 的 YAML 和地图图像哈希，不需要手动编辑哈希。如果路线文件已经
包含旧地图的点位，程序会拒绝换绑；此时必须使用当前地图重新创建一个新的
路线文件，不能用 `--force-capture` 或 `--update` 绕过检查。

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
tree_offset_arm_base_m 使用 alicia_base_link 坐标，不能直接套用车体坐标。
当前 alicia_mount_joint 相对车体绕 Z 轴旋转 pi：
alicia +X = 车体 -X，alicia +Y = 车体 -Y。
```

`point_2` 和 `point_3` 的树坐标必须使用机械臂基座坐标系下的带符号 X/Y。
关节预设观察模式支持 `tree-y-m > 0.05` 的左侧树和 `tree-y-m < -0.05` 的右侧树；
这是 `alicia_base_link` 的正/负 Y，不是车体正/负 Y。当前安装下，不能因为树位于
车体某一侧而手工猜测符号；路线管理器会应用安装偏航 `yaw_rad: pi` 后再生成 map 中的树提示。

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

## 6. 先验证导航与继电器联调

五点采集完成后，先验证真实小车导航和两个继电器的配合，不要立即启动真实机械臂、C10 和 YOLO。

采点时启动的 `real_navigation.launch.py` 必须先停止。联调入口会重新启动 Nav2；同时运行两个 Nav2、AMCL 或 map server 会造成节点、TF 和导航 Action 冲突。

注意：`real_navigation.launch.py` 只提供底盘、定位和 Nav2，不启动
`controller_pkg`，因此单独运行它时不存在 `/relay/set` 是正常现象。继电器单测应单独启动
`controller_pkg/launch/controller.launch.py`，完整联调应使用下面的
`real_field_route_validation.launch.py`。

这两种启动方式不能同时运行：`real_field_route_validation.launch.py` 已经内置
`controller.launch.py`。如果先启动了单独的 `controller.launch.py`，必须先在该终端
按 `Ctrl+C` 退出，再启动联调入口；否则两个进程会同时打开同一个 Modbus 串口，造成
`应答长度错误`、继电器控制失败或两个节点争抢 `/relay/set`。

### 6.1 单独验证继电器

```bash
ros2 service type /relay/set
```

应返回：

```text
wvcsc_interfaces/srv/SetRelay
```

首次测试必须断开喷头或关闭压力源。服务成功只表示 Modbus 写线圈成功，不代表水泵压力或药液已经喷出。

测试第1路：

```bash
ros2 service call /relay/set \
  wvcsc_interfaces/srv/SetRelay \
  "{channel: 1, enabled: true, duration: 3.0}"
```

测试第2路：

```bash
ros2 service call /relay/set \
  wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: true, duration: 3.0}"
```

确认继电器指示灯或触点动作，3 秒后控制器自动断开，并在 `relay_controller` 日志中看到自动断开记录。测试结束后显式关闭：

```bash
ros2 service call /relay/set \
  wvcsc_interfaces/srv/SetRelay \
  "{channel: 1, enabled: false, duration: 0.0}"
ros2 service call /relay/set \
  wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: false, duration: 0.0}"
```

### 6.2 五点导航与继电器联调

启动独立联调入口：

```bash
ros2 launch wvcsc_bringup real_field_route_validation.launch.py \
  relay_config_file:="$(ros2 pkg prefix controller_pkg)/share/controller_pkg/config/fault.ini" \
  use_rviz:=true
```

联调入口启动后不要再次执行 `ros2 launch controller_pkg controller.launch.py`。
它会自动启动唯一的继电器节点，并等待操作者在 RViz 设置 `2D Pose Estimate`。
在收到 `/amcl_pose` 之前不会打开第1路，也不会发送第一个导航目标；初始定位完成
后由路线管理器自动开始五点流程。

该入口启动真实底盘、LiDAR、IMU、EKF、Nav2、真实 `/relay/set` 和同一个 `field_route_manager`，但不启动真实机械臂、MoveIt、C10、YOLO 或 Visual Servo。`fake_arm_spray_action.py` 在 `point_2`、`point_3` 模拟机械臂完成，并真实驱动第2路各 3 秒。

联调顺序：

```text
第1路开启 → 导航 point_1 → 继续导航 point_2
point_2：第1路关闭 → 车辆停稳 → 模拟第2路喷洒3秒
      → 第2路关闭 → 第1路重新开启 → 导航 point_3
point_3：第1路关闭 → 车辆停稳 → 模拟第2路喷洒3秒
      → 第2路关闭 → 第1路重新开启 → 导航 point_4
point_4：第1路关闭 → 导航 point_5
point_5：第1、2路关闭 → 车辆停稳 → 完成
```

预期日志至少包含：

```text
[FIELD_ROUTE] services ready; auto-starting mission=...
[FIELD_ROUTE] arrived at point_1
[FIELD_ROUTE] arrived at point_2
[FAKE_ARM] inspect=1 ... channel=2 duration=3.0s
[FIELD_ROUTE] arrived at point_3
[FAKE_ARM] inspect=2 ... channel=2 duration=3.0s
[FIELD_ROUTE] arrived at point_4
[FIELD_ROUTE][SUCCESS] five-point route completed
```

监控命令：

```bash
ros2 topic echo /mission/status
ros2 action list
ros2 service type /relay/set
```

联调通过只证明导航、继电器和任务编排正确，不代表机械臂视觉喷洒已经通过。

## 4. 兼容模式：启动完整五点实车任务

联调入口验收完成后先按 `Ctrl-C` 停止
`real_field_route_validation.launch.py`，确认第1、2路都已断开，再启动生产任务。

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
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python" \
  relay_config_file:="$(ros2 pkg prefix controller_pkg)/share/controller_pkg/config/fault.ini"
```

不传 `relay_config_file` 时，使用安装包中的 `controller_pkg/config/fault.ini`。
手眼标定默认自动选择 config 目录下最新的
`c10_handeye_YYYYMMDD_HHMMSS.calib`。
地图和任务文件也会自动选择各自最新的时间戳目录；只有测试旧版本时，才显式传入
`mission_file:=...` 或 `map:=...` 覆盖默认值。

喷嘴默认读取：

```text
$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/nozzle.example.yaml
```

该文件是临时 tool0 零偏置配置，`spray_nozzle_link` 与 `tool0` 平移为零、旋转为单位
旋转，不代表真实喷嘴外参。

## 5. 五点兼容任务行为

```text
point_1:
  第1路广域喷洒开启 → 导航

point_2:
  第1路关闭 → 车辆停稳 → 机械臂识别病害并通过第2路喷洒3秒
  → 第1路重新开启

point_3:
  第1路关闭 → 车辆停稳 → 机械臂识别病害并通过第2路喷洒3秒
  → 第1路重新开启 → 继续导航

point_4:
  到达后关闭第1路 → 继续导航

point_5:
  第1、2路再次关闭 → 车辆停稳 → 任务完成
```

车辆到达 `point_2`、`point_3` 和 `point_5` 后，仍会执行停稳检查；该检查是运行安全门控，避免车辆运动时机械臂动作，不属于采点质量门控。

任务采用“单点失败跳过、路线继续”策略：

1. 第 1、2 路继电器请求超时、拒绝、通信异常或无法确认关闭时，只记录
   `[FIELD_ROUTE][WARN][RELAY]`，立即继续原定步骤；此时继电器实际状态未知，必须由现场
   急停和物理断电兜底。
2. `OBSERVE_FAILED`、`VISION_FAILED`、车辆停稳检查失败，以及已收到明确结果的 Nav2
   失败，会记录为 `[FIELD_ROUTE][SKIPPED]` 并跳过当前点位；`/mission/status` 的
   `skipped_targets` 会增加。
3. 机械臂 Action 超时、取消、锁定、回 HOME 失败、喷洒失败、Action 结果通信失败，以及
   启动前 Nav2/AMCL/TF 未就绪仍会取消活动 Action、最佳努力关闭两路继电器，并发布
   `FAILED`。这些情况无法确认前序运动是否已经结束，不能与下一导航目标并发。

## 9. 运行监控

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

## 10. 取消与急停

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

## 11. 当前已移除的旧实机入口

以下入口不再安装或用于实车主流程：

- `load_site_mission.py`；
- `nav_validate_sites.py`；
- `/mission/load_manual` 作为实车任务启动接口；
- 旧 measured-site mission 逐树自动任务流程。

仿真仍保留 `wvcsc_mission_manager`，因此不要从工作区删除该 ROS 包。

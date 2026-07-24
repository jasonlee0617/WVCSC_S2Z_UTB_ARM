# WVCSC 实机 Bringup

所有命令均需先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

## Qt 任意路线任务（默认实车入口）

完整实车任务默认启动 Qt 编辑器，且在操作者点击开始前**不会自动行驶**：

```bash
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

不要同时启动 `real_navigation.launch.py`；完整入口已包含同一套 Nav2/AMCL。先在
RViz 点击 `2D Pose Estimate` 完成定位，再在 Qt 点击“记录起点”。路线编辑规则：

- 通行点/终点：一次 `2D Goal` 作为停车位；
- 病株检查点：第一次 `2D Goal` 为停车位，第二次 `2D Goal` 点树中心；Qt 计算
  `alicia_base_link` 的带符号树相对 X/Y；
- `tree_y > 0.05` 使用左侧预设，`tree_y < -0.05` 使用右侧预设，中心线附近拒绝；
- “驶向该点时开启广域喷洒”是入段第1路属性；病株到点后第1路关闭，第2路由机械臂
  喷洒；
- 支持表格排序、删除、逐点喷洒/停留设置、JSON 保存加载和完成后返回起点；
- “终止作业并回HOME”通过 `/mission/abort_and_home` 取消导航/机械臂，关闭两路并让
  `motion_control` 安全复位。

当前 LiDAR 不自动输出单株玉米中心或病株 ID，因此病株树中心仍需人工在 RViz 选择。
单点任务错误会标记跳过并继续；动作超时、未确认取消、运动锁定和关键定位缺失仍会停止。

使用原五点 YAML 路线而非 Qt 时，显式选择兼容模式：

```bash
ros2 launch wvcsc_bringup real_system_mission.launch.py mission_mode:=file
```

## 1. 建图

```bash
ros2 launch wvcsc_bringup real_cartographer.launch.py
```

此命令复用已验证的底盘、LiDAR、IMU、EKF 链，启动 Cartographer 和
`wvcsc_bringup/rviz/real_cartographer.rviz`，不启动 C10、Nav2、机械臂或任务节点。完成建图后保存为
`maps/map_YYYYMMDD_HHMMSS/orchard.{yaml,pgm}`：

```bash
bash $(ros2 pkg prefix wvcsc_bringup)/share/wvcsc_bringup/scripts/save_corn_map.sh
```

脚本默认创建时间戳目录；也可将其他输出基名作为第一个参数传入。

## 2. 实机导航

一个命令启动底盘、LiDAR、IMU、EKF、map_server、AMCL、Nav2 和一个 RViz；不启动 C10：

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py
```

默认自动选择最新时间戳地图。使用其他地图时显式传入绝对路径：

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py \
  map:="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/map_YYYYMMDD_HHMMSS/other.yaml"
```

该启动链与已通过实机验证的
`my_navigation2/launch/wtb_navigation2_fdimu.launch.py` 使用相同的硬件链、
Nav2 参数和 `tf_buffer_size`；硬件子 RViz 被关闭，只启动复制到
`wvcsc_bringup/rviz/real_navigation.rviz` 的导航 RViz。完整任务才加载
`real_sensors.launch.py` 和 C10。Nav2 速度平滑器的最终输出直接发布到
`/cmd_vel`；底盘紧急停止由硬件负责。

当前实车使用 Yesense IMU。首次部署或更换工控机时，先安装
`/dev/yesense_IMU` 稳定串口别名：

```bash
cd "${HOME}/WVCSC_S2Z_UTB_ARM/src/yesense_ros2"
sudo ./yesense_udev.sh
ls -l /dev/yesense_IMU
```

随后启动导航，确认 Yesense 节点日志显示串口打开成功，并确认
`ros2 topic hz /imu` 持续有数据。串口路径由
`yesense_std_ros2/config/yesense_config.yaml` 中的
`/dev/yesense_IMU` 管理，不要写死 `ttyUSB0` 或 `ttyACM0`。

旧版 `fdilink_ahrs` 代码和包仍保留，但已从默认启动链停用；只有回滚到旧 IMU
时才恢复对应的注释启动块，禁止两个驱动同时运行。

## 3. 旧共享逐树实测停靠点

这一节的 schema-v3 文件仍供仿真、Nav2 Qt 和 `/mission/load_manual`
使用；它不是新的实机五点作业入口。

AMCL稳定后，人工驾驶并停稳。卷尺必须从机械臂基座物理原点测量，坐标轴
保持与车体平行：`+X`为车头前方、`+Y`为车体左侧。树在机械臂基座正侧方时
`--tree-x-m`填`0.0`，允许的纵向误差为`±0.20 m`：

已有的 schema-v2 站点文件可迁移，schema-v1 文件必须重新采集：

旧共享站点必须由操作者显式指定；地图默认取最新时间戳地图：

```bash
SITE_FILE="<existing-schema-v2-site.yaml>"
MAP_FILE="$(python3 -c 'from wvcsc_bringup.path_defaults import latest_map_yaml; print(latest_map_yaml())')"
```

```bash
ros2 run wvcsc_bringup migrate_site_mission -- \
  --file "$SITE_FILE" \
  --map "$MAP_FILE"
```

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$SITE_FILE" \
  --map "$MAP_FILE" \
  --capture-home

ros2 run wvcsc_bringup capture_site_pose -- \
  --file "$SITE_FILE" \
  --map "$MAP_FILE" \
  --target-id corn_01 \
  --tree-x-m 0.0 --tree-y-m <带符号实测值> \
  --spray-duration 5.0
```

重复目标默认拒绝覆盖，确认更新时增加`--update`。完成全部目标后验证：

```bash
ros2 run wvcsc_bringup validate_site_mission -- \
  --file "$SITE_FILE" \
  --map "$MAP_FILE"
```

任务文件使用schema v2，并与地图YAML及图片SHA256绑定；地图改变后旧任务会被拒绝。
schema v1以`base_footprint`为测量原点，不能自动转换。升级后先备份旧
`corn_site.yaml`，再依次重新采集HOME和每棵树。
采点脚本会自行调用 AMCL 的 `/request_nomotion_update`，无需额外终端循环调用该
服务；当前只要求流程成功，采点质量门限临时放宽为位置/偏航散布 ≤ 1.00 m/rad、
AMCL 位置/偏航标准差 ≤ 1.00 m/rad。门限集中定义在
`wvcsc_bringup/wvcsc_bringup/site_mission.py`，修改后重新构建并 source 工作区。
定位链稳定后应恢复严格门限；放宽门限不代表当前定位精度满足最终工程验收。
采样期间 AMCL 位姿允许最多 2 秒未更新；短暂过期时脚本会自动等待并重试，
不会因约 1 Hz 发布抖动直接中断采点。

如果当前阶段只要求先写入站点文件，可临时增加 `--force-capture`：

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --map "$MAP_FILE" \
  --target-id tree_01 --tree-x-m 0.0 --tree-y-m 1.60 \
  --timeout-sec 60 --force-capture
```

该模式仅保留初始 `/imu`、`/ekf_odom`、`/amcl_pose` 和 30 个有效 TF 样本要求，
跳过新鲜度、停稳、质量和地图 footprint 门控；仅用于当前调试，不代表站点位姿可靠。

### 3.1 兼容的实机五点路线采集

这是 `mission_mode:=file` 的兼容入口；默认 Qt 任意路线不读取这个五点 YAML。
现场逐点停稳后，用同一个文件依次采集 `point_1` 至 `point_5`。点 2、3 的树位置
树偏移以 `alicia_base_link` 为原点，而不是车体坐标。当前 `alicia_mount_joint`
相对车体绕 Z 轴旋转 `pi`，因此 `alicia +X = 车体 -X`、`alicia +Y = 车体 -Y`。
关节预设模式支持 `tree-y-m > 0.05` 的左侧和 `tree-y-m < -0.05` 的右侧；不要按车体
左右方向手工反转符号。两点喷洒时长固定为 3.0 s。

```bash
MISSION_DIR="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/real/mission_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$MISSION_DIR"
ROUTE_FILE="$MISSION_DIR/field_route_corn.yaml"
MAP_FILE="$(python3 -c 'from wvcsc_bringup.path_defaults import latest_map_yaml; print(latest_map_yaml())')"
cp "${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/real/field_route_corn.example.yaml" "$ROUTE_FILE"

ros2 run wvcsc_bringup capture_site_pose -- --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_1
ros2 run wvcsc_bringup capture_site_pose -- --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_2 --tree-id corn_01 --tree-x-m 0.0 --tree-y-m 1.20
ros2 run wvcsc_bringup capture_site_pose -- --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_3 --tree-id corn_02 --tree-x-m 0.0 --tree-y-m 1.20
ros2 run wvcsc_bringup capture_site_pose -- --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_4
ros2 run wvcsc_bringup capture_site_pose -- --file "$ROUTE_FILE" --map "$MAP_FILE" --route-point point_5
ros2 run wvcsc_bringup validate_field_route -- --file "$ROUTE_FILE" --map "$MAP_FILE"
```

不要把图中约 8 m、2 m 的示意值直接写成导航坐标；采点工具会写入地图哈希、五点
位姿和每点质量记录。地图改变后，旧路线将被前置检查拒绝。

## 硬件默认值与标定文件

当前代码默认按小车工控机配置：

| 项目 | 开发机常用覆盖值 | 小车默认值 |
| --- | --- | --- |
| C10 V4L2 | `/dev/video0` | `/dev/video2` |
| Alicia-M | `/dev/ttyACM0` | `/dev/ttyACM0` |
| 继电器配置 | `fault_dev.ini` | `controller_pkg/config/fault.ini` |
| 实机手眼标定 | config 下最新 `c10_handeye_*.calib` | 同左 |
| 仿真手眼标定 | config 下最新 `c10_handeye_sim_*.calib` | 不使用实机文件 |

开发机设备不同时间，通过 `c10_device:=...`、`serial_port:=...` 和
`relay_config_file:=...` 覆盖；C10 是视频设备路径，不是串口。运行前可用
`v4l2-ctl --list-devices`、`ls -l /dev/serial/by-id /dev/serial/by-path` 核对设备。
实机和仿真标定结果均写入 `$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config`，
每次生成的 `.calib` 与 `.yaml` 使用同一 `YYYYMMDD_HHMMSS` 后缀，启动时选择最新匹配文件。

## 4. 机械臂单独喷洒测试

此模式用于现场单独验证机械臂喷洒闭环：C10、真实 YOLO、MoveIt、Servo、
VisualServo、SprayTask 和喷洒 Action。它不启动底盘、LiDAR、IMU、EKF、Nav2
或 MissionManager；车辆必须人工停稳，底盘电机保持不可运动状态。

先确认真实权重已经放入 `wvcsc_rgb_vision/models/`：

- `yolov8s_real.pt`: `detect`, `{0: tree}`；
- `yolov8s_seg_real.pt`: `segment`, `{0: disease_leaf}`。

启动前还会读取实机标定：默认从
`$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config` 自动选择最新的
`c10_handeye_YYYYMMDD_HHMMSS.calib`；也可通过
`handeye_calibration:=/绝对路径/文件.calib` 显式指定。
- 喷洒测试暂时将 `tool0` 作为喷洒中心线，喷嘴挂载为零位姿，不读取喷嘴标定文件。

启动机械臂单独测试栈：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

另开终端，按机械臂基座 `alicia_base_link` 测量玉米树位置后发送一次喷洒目标。
坐标约定与采点一致：`+X` 为车头前方，`+Y` 为车体左侧。玉米树放在机械臂正左侧
时 `--tree-x-m` 填 `0.0`：

实机默认 `observation_mode:=joint_presets`：MoveIt 对左、右两侧分别按“正对、扇形、扇形”
三组已现场确认的关节姿态扫描，`tree_y_m > 0.05` 为左侧、`tree_y_m < -0.05` 为右侧。当前
`spray_working_distance_m=1.00 m` 是这三组姿态的人工确认作业距离；不是由初始
观察 IK 计算得出。中心线附近（`abs(tree_y_m) <= 0.05`）会在机械臂运动前明确拒绝；
也可显式改用原有 IK 观察模式：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  observation_mode:=ik \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

IK 模式才要求树到机械臂基座的二维距离**大于 `1.05 m`**，否则相机在基座与树之间
没有生成 `1.00 m` 观察位姿的空间。建议使用 `|--tree-y-m| = 1.50 m`。

```bash
ros2 run wvcsc_bringup arm_spray_once -- \
  --target-id corn_01 \
  --tree-x-m 0.0 \
  --tree-y-m 1.50 \
  --spray-duration 5.0
```

检查话题：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /mission/status --once
ros2 topic echo /vision/inference_mode
ros2 topic hz /vision/tree_debug_image
ros2 topic hz /vision/diseased_target_debug_image
ros2 topic echo /vision/target
ros2 topic echo /vision/visual_servo_debug
ros2 topic echo /spray/simulated_active
```

通过标准：日志依次出现 `MOVING_TO_OBSERVE`、`SCANNING_TREE`、
`DETECTING_FRUITS`、`QUEUING`、`ALIGNING`、`SPRAYING`、
`RETURNING_TO_OBSERVE`、`RETURNING_HOME`；`/vision/tree_debug_image` 能看到
玉米树框，`/vision/diseased_target_debug_image` 能看到 `diseased_target` 标注，
`/vision/visual_servo_debug` 返回成功，对应喷洒期间
`/spray/simulated_active` 为 `true`。

当前实机入口默认通过 `controller_pkg` 的 `/relay/set` 控制第 2 路虫害喷洒
继电器。实机 `spray_actuator_real.yaml` 固定使用 `spray_mode: service`，不会退回
仿真的 `timer` 模式。启动前必须确认 `controller_pkg/config/fault.ini` 的继电器
串口、波特率和从站地址与实际设备一致；小车默认继电器路径为
`/dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.0-port0`。这里的继电器串口与上面的机械臂
`serial_port` 是两条独立串口，不能因为机械臂使用 `/dev/ttyACM0` 就把继电器也改成
同一个设备。

在小车工控机上先验证服务和继电器，再启动完整测试：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 service type /relay/set
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: true, duration: 1.0}"
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: false, duration: 0.0}"
```

确认第 2 路实际动作后，再启动：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python" \
  serial_port:=/dev/ttyACM0 \
  relay_config_file:="$(ros2 pkg prefix controller_pkg)/share/controller_pkg/config/fault.ini"
```

`relay_config_file` 不传时使用安装包中的
`controller_pkg/config/fault.ini`。喷洒 Action 开始时，执行器会先等待
`/relay/set` 返回 `success=true`，请求第 2 路并带上喷洒时长；服务端还会按该时长
自动断开，Action 结束和取消路径也会显式断开第 2 路。

## 5. 完整定位与作业

```bash
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

默认 `mission_mode:=qt` 会启动 Qt 编辑器，但在你完成 RViz 初始定位并点击 Qt 的开始按钮前
不会导航。启动前检查会在任何硬件节点运行之前验证：

- C10 和 Alicia-M 设备路径；
- 地图文件和 ROS 包；
- 当前地图、Qt/任务 ROS 包与停车相关基础配置；
- 独立 YOLO Python 环境；
- `yolov8s_real.pt`: `detect`, `{0: tree}`；
- `yolov8s_seg_real.pt`: `segment`, `{0: disease_leaf}`。
- `wvcsc_c10_camera/config/c10_intrinsics.yaml`；
- `$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config` 中最新的
  `c10_handeye_YYYYMMDD_HHMMSS.calib`；
- `${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/nozzle.example.yaml`。

实机权重需由用户放入
`wvcsc_rgb_vision/models/`。权重缺失或类别契约不匹配时，整个实机启动会
在硬件上电前失败，不会回退到仿真权重。

若要运行旧五点 YAML，请显式传入 `mission_mode:=file`；这时前置检查才会校验
schema-v4 五点路线和地图哈希，并在就绪后自动开始：

1. 进入第 1 点前接通第 1 路广域喷洒；
2. 第 2、3 点到达后断开第 1 路，确认车辆停稳，机械臂识别病害并只通过第 2 路喷洒 3.0 s；
3. 每次机械臂成功后重新接通第 1 路并驶向下一点；第 4 点关闭第 1 路；
4. 第 5 点再次关闭第 1、2 路，确认停稳后结束。

继电器请求超时、拒绝或通信异常只会输出 `[FIELD_ROUTE][WARN][RELAY]`，并按原定步骤继续；
继电器的实际吸合状态此时无法由软件保证。`OBSERVE_FAILED`、`VISION_FAILED`、车辆停稳检查
失败，以及已收到明确结果的 Nav2 失败会跳过当前点位，记录到 `/mission/status` 的
`skipped_targets` 后继续路线。机械臂 Action 超时、取消、锁定、回 HOME 失败、喷洒失败、
Action 结果通信失败以及启动前 Nav2/定位未就绪仍以 `FAILED` 结束，避免与未知中的运动并发。
运行中查看 `/mission/status`；人工取消使用：

```bash
ros2 service call /field_route/cancel std_srvs/srv/Trigger "{}"
```

## 6. 停止与恢复

停止五点任务时先调用 `/field_route/cancel` 或终止 launch，确认 `/cmd_vel` 已归零，再使用机械臂
控制命令：

```bash
ros2 service call /field_route/cancel std_srvs/srv/Trigger "{}"
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: reset}"
# 等待 /motion_control/state 显示 HOME_LOCKED 后再恢复。
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: resume}"
```

`motion_control_keyboard` 只提供空格停止、`h` 回 HOME、`r` 恢复；不会自动
取消底盘导航。实车运行必须保留可触达的底盘物理急停，并由操作员确认车辆完全停止。

## 7. 标定

内参（8x6 内角点，25 mm）：

```bash
ros2 launch wvcsc_c10_camera c10_intrinsics.launch.py
```

手眼标定（`DICT_5X5_250`, ID 1, 70 mm）使用三个终端：

```bash
# 终端一：硬件、MoveIt、C10、ArUco、easy_handeye2服务
ros2 launch wvcsc_calibration c10_handeye.launch.py

# 终端二：Alicia-M自适应自动采集；输入s或空Enter开始，q取消
ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file $(ros2 pkg prefix wvcsc_calibration)/share/\
wvcsc_calibration/config/auto_handeye_alicia.yaml

# 终端三：SPACE/h/r/x安全介入
ros2 run wvcsc_arm_task motion_control_keyboard
```

手眼标定 launch 不加载 `real_sensors.launch.py`：不会启动底盘、LiDAR、IMU、EKF
或 Nav2。车辆必须保持停稳，并保留可触达的物理急停。

默认手眼输出为 config 目录下同一时间戳的一对文件：
`c10_handeye_YYYYMMDD_HHMMSS.calib` 与
`c10_handeye_YYYYMMDD_HHMMSS.yaml`。
自动采集会先根据 marker 相对 `alicia_base_link` 的配置生成安全初始观察位，
再清空上一轮服务端样本，逐候选执行碰撞IK、Jacobian条件数、关节余量和OMPL
门控，并以Park/Horaud/Tsai-Lenz共识及离群剔除后结果原子写入。

当前临时联调直接使用工作区中的 tool0 零偏置配置：

```bash
NOZZLE_FILE="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/nozzle.example.yaml"
test -f "$NOZZLE_FILE"
```

`nozzle.example.yaml`描述`tool0→spray_nozzle_link`，当前平移为零、旋转为单位旋转，
固定工距为`1.00±0.05m`。它只是临时 tool0 假设，不代表真实喷嘴外参；完成真实喷嘴
标定后应替换该文件。

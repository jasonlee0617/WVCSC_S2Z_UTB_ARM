# WVCSC 实机 Bringup

设备通讯口、udev 别名、物理 USB 插口更换后的处理，以及从建图到完整任务的可复制命令，
统一维护在 `docs/实机硬件通讯口与设备检查指南.md` 和
`docs/小车机械臂完整任务操作指南.md`。本 README 保留启动链路说明，避免硬件检查命令
在两处长期漂移。

先加载环境：

```bash
cd ~/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

## 实机启动前硬件检查命令

下面的检查应在启动完整任务前单独执行。除特别标注的继电器“断开”请求外，命令均为
只读检查。完整任务启动时也会自动运行 `preflight_check.py`；终端出现 `[FAIL]` 时不会
启动机械臂、底盘或继电器链路。

### 1. 先查看 USB、视频和串口设备

```bash
# 查看 USB 设备以及内核最近识别结果
lsusb
sudo dmesg --ctime | tail -n 80

# C10 是 USB/V4L2 视频设备，不是串口；机械臂和 Yesense 才是串口设备
ls -l /dev/video* /dev/ttyACM* /dev/ttyUSB* \
  /dev/serial/by-id /dev/serial/by-path 2>/dev/null || true
```

不要只凭 `/dev/my_robot` 的编号判断设备身份；设备重新插拔后编号可能变化。应结合
`udevadm info`、`/dev/serial/by-id` 或 `/dev/serial/by-path` 确认。

### 2. C10 相机（视频设备）

当前实机默认设备是 `/dev/video2`，默认话题是 `/camera/color/image_raw`：

```bash
CAMERA_DEVICE=/dev/video2
test -e "$CAMERA_DEVICE" && echo "camera device exists" || echo "camera device missing"
v4l2-ctl --list-devices
v4l2-ctl --device="$CAMERA_DEVICE" --all
v4l2-ctl --device="$CAMERA_DEVICE" --list-formats-ext
udevadm info --query=all --name="$CAMERA_DEVICE" | rg 'ID_VENDOR|ID_MODEL|ID_SERIAL|DEVNAME'
fuser -v "$CAMERA_DEVICE" || true
```

确认设备节点后，单独启动相机并检查 ROS 输出。该命令会启动相机驱动和看门狗，结束
检查时按 `Ctrl-C`：

```bash
ros2 launch wvcsc_c10_camera c10_camera.launch.py \
  video_device:="$CAMERA_DEVICE"

ros2 topic info /camera/color/image_raw --verbose
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

期望看到 `sensor_msgs/msg/Image`、640×480 的 `CameraInfo` 和接近 30 Hz 的图像流。
如果 `fuser` 显示已有相机驱动占用，应先停止旧的相机节点，避免两个 `usb_cam` 同时打开
同一个设备。

### 3. Alicia-M 机械臂串口

当前实机默认机械臂串口是 `/dev/my_robot`，默认波特率为 `1000000`：

```bash
ARM_DEVICE=/dev/my_robot
ls -l "$ARM_DEVICE"
udevadm info --query=all --name="$ARM_DEVICE" | rg 'ID_VENDOR|ID_MODEL|ID_SERIAL|DEVNAME'
readlink -f "$ARM_DEVICE"
fuser -v "$ARM_DEVICE" || true

# 只查看启动参数，不连接机械臂
ros2 launch wvcsc_bringup real_arm.launch.py --show-args
```

确认没有其它程序占用串口后，再进行一次独立驱动连通性检查：

```bash
ros2 launch wvcsc_bringup real_arm.launch.py \
  serial_port:="$ARM_DEVICE" baudrate:=1000000 use_rviz:=false

# 在另一个终端检查控制器和关节状态；此检查不会主动发送喷洒 Goal
ros2 control list_controllers
ros2 topic info /joint_states --verbose
ros2 topic hz /joint_states
```

不要在完整任务已经运行时重复启动 `real_arm.launch.py`，否则会争抢同一个机械臂串口。
检查过程中不要直接向串口写入数据；复位和运动应由 Qt 或既有机械臂节点执行。

### 4. 继电器串口和配置

继电器使用 Modbus RTU。完整任务默认读取 `controller_pkg/config/fault.ini`，当前项目
约定通道 1 为广域喷洒、通道 2 为机械臂喷嘴：

```bash
RELAY_CONFIG="$(ros2 pkg prefix controller_pkg)/share/controller_pkg/config/fault.ini"
sed -n '/^\[serial\]/,/^\[/p' "$RELAY_CONFIG"
grep -E '^(PortName|BaudRate|Address|Timeout)[[:space:]]*=' "$RELAY_CONFIG"

# 解析配置中的串口后检查设备是否存在（配置中可能是 by-path 或 ttyUSB）
RELAY_DEVICE=$(awk -F= '/^[[:space:]]*PortName[[:space:]]*=/{gsub(/[[:space:]]/, "", $2); print $2; exit}' "$RELAY_CONFIG")
echo "relay device: $RELAY_DEVICE"
ls -l "$RELAY_DEVICE"
udevadm info --query=all --name="$RELAY_DEVICE" | rg 'ID_VENDOR|ID_MODEL|ID_SERIAL|DEVNAME' || true
fuser -v "$RELAY_DEVICE" || true
```

启动服务并确认服务类型：

```bash
ros2 launch controller_pkg controller.launch.py \
  config_file:="$RELAY_CONFIG"

ros2 service list | rg '^/relay/set$'
ros2 service type /relay/set
ros2 param get /relay_controller config_file
ros2 interface show wvcsc_interfaces/srv/SetRelay
```

确认泵和喷洒装置处于安全状态后，可只发送“断开”请求验证软件命令路径。该请求会改变
继电器状态，不要在喷洒过程中执行：

```bash
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 1, enabled: false, duration: 0.0}"
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: false, duration: 0.0}"
```

返回 `success: true` 只代表 Modbus 命令收到并通过软件校验，不等价于已经测得继电器
触点或泵的实际流量。

### 5. Yesense IMU 串口和 `/imu` 话题

当前实机 IMU 驱动是 `yesense_std_ros2`，默认串口别名 `/dev/yesense_IMU`、波特率
`460800`，标准输出是 `/imu`。不要同时启动已停用的 `fdilink_ahrs`，否则会产生重复
`/imu` 发布者：

```bash
IMU_DEVICE=/dev/yesense_IMU
ls -l "$IMU_DEVICE"
udevadm info --query=all --name="$IMU_DEVICE" | rg 'ID_VENDOR|ID_MODEL|ID_SERIAL|DEVNAME'
readlink -f "$IMU_DEVICE"
fuser -v "$IMU_DEVICE" || true

ros2 launch yesense_std_ros2 yesense_node.launch.py
```

在另一个终端确认消息、发布者和频率：

```bash
ros2 topic info /imu --verbose
ros2 topic echo /imu --once
ros2 topic hz /imu
ros2 topic echo /imu_data_extend --once
```

### 6. CAN 盒和底盘数据链

当前 `can_bridge` 使用厂商 VCI CAN 盒（源码固定 `DevType=4`、设备索引 0、两个通道），
不是 SocketCAN 接口。因此没有 `/dev/can0` 时不代表故障，`ip link show can0` 不能作为
本项目 CAN 盒的唯一检查依据：

```bash
lsusb
ros2 launch can_bridge can_bridge.launch.py
```

确认节点和 ROS CAN 话题：

```bash
ros2 node list | rg 'can_bridge_node'
ros2 node info /can_bridge_node
ros2 topic list -t | rg '/can_(rx|tx)_[12]'
ros2 topic info /can_tx_1 --verbose
ros2 topic info /can_tx_2 --verbose
ros2 topic echo /can_tx_1
```

`/can_tx_1`、`/can_tx_2` 是 CAN 盒接收后发布到 ROS 的话题，`/can_rx_1`、`/can_rx_2` 是
ROS 发往 CAN 盒的输入话题。只有总线上确实有帧时 `ros2 topic hz /can_tx_1` 才会有输出；
没有帧不能单独判定 CAN 盒掉线。启动终端应重点检查“打开设备失败”“通道初始化失败”或
“设备掉线”等日志。

### 7. LiDAR、里程计、EKF 和 TF

底盘导航至少需要点云、轮速里程计、IMU 和 EKF 输出：

```bash
ros2 node list | rg 'lslidar|wtb_car|ekf_filter_node|pointcloud_to_laserscan|yesense'
ros2 topic info /point_cloud_raw --verbose
ros2 topic hz /point_cloud_raw
ros2 topic echo /car_odom --once
ros2 topic echo /ekf_odom --once
ros2 topic hz /ekf_odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

如果 `/imu`、`/car_odom` 或 `/point_cloud_raw` 没有数据，先修复对应硬件链路，再启动
Nav2；不要只根据 RViz 窗口已经打开就判断导航输入正常。

## 完整任务：Qt 选点、导航、喷洒

```bash
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

完整任务只接受 Qt 创建的任务：不再支持 YAML/命令行采点和 `mission_mode`。在 RViz 设置
`2D Pose Estimate` 后，在 Qt 点击“确认当前起点”。通行点或终点用一个 `2D Goal` 记录；
病株点先点击停车位，再点击树中心。Qt 可保存/加载 JSON 任务。

初始定位不准或误记点位时，点击“重新定位并清空任务”。它会清空起点和任务列表、请求 AMCL
全局重定位；随后重新在 RViz 点击 `2D Pose Estimate`，再确认新起点。

完整 Qt 的下方右侧是原始 `sensor_msgs/Image` 图像查看器，会动态列出相机和 YOLO 图像话题，
优先显示：

- `/camera/color/image_raw`
- `/vision/diseased_target_debug_image`

视觉伺服超时、目标过期或输出停滞可按实机配置执行受控回退喷洒；伺服硬安全停止、碰撞、
关节限位及奇异性门控不能绕过。

## 真实底盘与继电器联调（Qt + 假机械臂）

```bash
ros2 launch wvcsc_bringup real_vehicle_relay_qt_test.launch.py
```

该入口启动真实底盘、LiDAR、IMU、EKF、Nav2 和真实继电器。Qt 负责选点与提交任务；
`fake_arm_spray_action.py` 代替真实机械臂，并实际脉冲第 2 路继电器。它不启动 MoveIt、C10、
YOLO 或视觉伺服。

## 单机械臂喷洒测试

默认启动真实单臂喷洒后端和 Qt：

```bash
cd ~/WVCSC_S2Z_UTB_ARM
./run_real_arm_spray_server.sh
```

该脚本只启动 launch，不会自动发送 `/arm/execute_spray` Goal、移动机械臂或打开继电器。
我在 Qt 中填写单目标参数并点击“启动”后，`wvcsc_spray_task` 才负责观测、检测、伺服、
喷洒、复检和 HOME。无界面排障时可显式关闭 Qt：

```bash
cd ~/WVCSC_S2Z_UTB_ARM
./run_real_arm_spray_server.sh use_qt_gui:=false
```

也可以用原始 launch 一次启动后端和 Qt：

```bash
cd ~/WVCSC_S2Z_UTB_ARM
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

单臂 Qt 只执行一个目标。两种模式都提供病株侧位、喷洒时长和手动工作距离；IK 模式额外
显示机械臂基座到病株的平面距离，`joint_presets` 模式不需要该距离。工作距离只用于喷嘴
瞄准平面标定，不替代机械臂碰撞、限位和奇异性检查，也不会写入完整导航任务。界面还提供
复位、HOME 成功后的自动就绪、Action 阶段、进度、结果和可折叠的相机/YOLO 图像。

## 建图与单独导航

```bash
cd ~/WVCSC_S2Z_UTB_ARM
./run_cartograph.sh

# 结束建图后，另一个终端执行纯导航
./run_nav2.sh
```

实机默认 C10 为 `/dev/video2`，Alicia-M 串口为 `/dev/my_robot`；其他设备通过 launch 参数
显式覆盖。完整任务默认使用 `vision_real_detect.yaml` 的 `best.pt` detect 模型；完整任务启动前
会检查地图、标定、YOLO 环境、相机、机械臂和继电器配置。

# WVCSC_S2Z_UTB_ARM 机械臂单独喷洒流程解耦测试指南

本文档用于现场单独验证机械臂喷洒闭环，不启动小车导航链。

适用目标：

- 小车停在现场固定位置，底盘不运动；
- 参照仿真，将玉米树放在机械臂左侧；
- 只测试 Alicia-M、C10、真实 YOLO、VisualServo、SprayTask 和喷洒 Action；
- 实机默认使用 `spray_actuator` 的 `service` 模式，通过
  `controller_pkg` 的 `/relay/set` 控制第 2 路继电器；仿真才使用 `timer` 模式。

## 1. 当前解耦边界

启动文件：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py
```

该启动文件会启动：

- C10 相机：`/camera/color/image_raw`、`/camera/color/camera_info`；
- Alicia-M 实机控制、MoveIt、MoveIt Servo；
- `wvcsc_rgb_vision/two_stage_yolo`，加载 `vision_real.yaml`；
- `wvcsc_visual_servo`；
- `wvcsc_arm_task/spray_task`；
- `wvcsc_arm_task/spray_actuator`。
- `controller_pkg/relay_controller`，提供 `/relay/set` 继电器服务。

该启动文件不会启动：

- 小车底盘驱动；
- LiDAR；
- IMU；
- EKF；
- AMCL；
- Nav2；
- MissionManager；
- `real_sensors.launch.py`；
- `real_navigation.launch.py`。

注意：因为不启动 `real_sensors.launch.py`，此模式会让 `real_arm.launch.py` 单独发布机器人 TF，即 `publish_robot_state:=true`。否则会出现只有机械臂 `joint_states`、但没有 `tool0/camera` TF 的隐蔽故障。

## 2. 坐标测量约定

本测试直接把树坐标传给 `/arm/execute_spray`，不经过地图和 Nav2。

树坐标的参考系为：

```text
alicia_base_link
```

坐标轴约定：

- `+X`：车头前方；
- `+Y`：车体左侧；
- `+Z`：向上；
- 玉米树放在机械臂正侧方时，`tree-x-m=0.0`；
- 玉米树距离机械臂基座左侧 1.50 m 时，`tree-y-m=1.50`；右侧填写负值。

现场摆放建议：

```text
tree-x-m       = 0.0
tree-y-m       = 1.50
tree-z-m       = 0.0
```

如果机械臂观察位姿规划失败，先不要调 VisualServo。优先检查树相对机械臂基座的位置是否过近、过远或不在左侧可达区域。

## 3. 启动前检查

所有终端先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

确认新增入口存在：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py --show-args
ros2 run wvcsc_bringup arm_spray_once -- --help
```

确认真实 YOLO 权重已经放入：

```text
~/WVCSC_S2Z_UTB_ARM/src/wvcsc_rgb_vision/models/yolov8s_real.pt
~/WVCSC_S2Z_UTB_ARM/src/wvcsc_rgb_vision/models/yolov8s_seg_real.pt
```

模型契约必须是：

```text
yolov8s_real.pt      task=detect  names={0: tree}
yolov8s_seg_real.pt  task=segment names={0: disease_leaf}
```

确认标定文件存在：

```bash
ls -lt "$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config"/c10_handeye_*.calib
```

当前单独喷洒入口暂时将 `tool0` 作为喷洒中心线，喷嘴挂载使用零位姿；因此本入口
不读取 `nozzle.yaml`。手眼标定文件缺失或格式错误时，launch 会在启动阶段失败，不能
继续执行喷洒。

确认 C10 设备：

```bash
ls -l /dev/v4l/by-id/
```

默认设备为：

```text
/dev/video2
```

如果现场设备名不同，启动时用 `c10_device:=...` 显式传入。

确认 Alicia-M 串口：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

默认机械臂串口为：

```text
/dev/ttyACM0
```

如果不同，启动时用 `serial_port:=...` 显式传入。

确认继电器串口配置。机械臂串口和继电器串口是两个独立设备：

```bash
sed -n '1,20p' ~/WVCSC_S2Z_UTB_ARM/src/controller_pkg/config/fault.ini
```

`fault.ini` 中的 `PortName` 必须指向继电器 Modbus 串口（默认
`/dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.0-port0`），`serial_port` 只指向 Alicia-M
机械臂串口（小车默认 `/dev/ttyACM0`）。现场建议使用 `/dev/serial/by-id/` 或
`/dev/serial/by-path/` 下的稳定设备名，并确认当前用户
具有串口访问权限。

先启动继电器服务或启动完整测试栈后，验证服务类型：

```bash
ros2 service type /relay/set
# 应为：wvcsc_interfaces/srv/SetRelay
```

## 4. 启动机械臂单独测试栈

终端一：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

可选参数：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python" \
  c10_device:=/dev/video2 \
  serial_port:=/dev/ttyACM0 \
  relay_config_file:="$(ros2 pkg prefix controller_pkg)/share/controller_pkg/config/fault.ini" \
  arm_velocity_scaling:=0.20 \
  arm_acceleration_scaling:=0.20 \
  use_moveit_rviz:=true
```

不传 `relay_config_file` 时，launch 使用安装包中的
`controller_pkg/config/fault.ini`。如果现场修改了源文件，必须重新构建
`controller_pkg`，或者直接通过 `relay_config_file` 指定外部配置文件。

现场首次测试建议保持：

```text
arm_velocity_scaling:=0.20
arm_acceleration_scaling:=0.20
```

不要一开始提高速度。先确认观察、识别、对准和 HOME 都能稳定完成。

## 5. 发送一次单树喷洒目标

终端二：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 run wvcsc_bringup arm_spray_once -- \
  --target-id corn_01 \
  --tree-x-m 0.0 \
  --tree-y-m -1.0 \
  --spray-duration 5.0
```

参数说明：

- `--target-id`：本次测试目标 ID，只用于日志和视觉目标关联；
- `--frame-id`：默认 `alicia_base_link`，不建议改；
- `--tree-x-m`：树相对机械臂基座的 X；
- `--tree-y-m`：树相对机械臂基座的 Y；正值表示左侧，负值表示右侧。
- `--tree-z-m`：树根高度，默认 `0.0`；
- `--spray-duration`：喷洒 Action 持续时间，默认建议 `5.0` 秒。

如果树离机械臂左侧 1.60 m：

```bash
ros2 run wvcsc_bringup arm_spray_once -- \
  --target-id corn_01 \
  --tree-x-m 0.0 \
  --tree-y-m 1.60 \
  --spray-duration 5.0
```

## 6. 现场观察话题

建议开第三个终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 topic hz /camera/color/image_raw
ros2 topic echo /mission/status --once
ros2 topic echo /vision/inference_mode
ros2 topic hz /vision/tree_debug_image
ros2 topic hz /vision/diseased_target_debug_image
ros2 topic echo /vision/target
ros2 topic echo /vision/visual_servo_debug
ros2 topic echo /spray/simulated_active
```

`/spray/simulated_active` 只是喷洒执行状态话题名称，实机物理输出不由该话题直接
控制。继电器实际控制接口是：

```bash
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: true, duration: 1.0}"
ros2 service call /relay/set wvcsc_interfaces/srv/SetRelay \
  "{channel: 2, enabled: false, duration: 0.0}"
```

首次测试必须确认第 2 路确实吸合和断开，再执行机械臂喷洒任务。

图像查看：

```bash
rqt_image_view
```

推荐依次查看：

```text
/camera/color/image_raw
/vision/tree_debug_image
/vision/diseased_target_debug_image
```

注意：进入 `SPRAYING` 后，`spray_task` 会把 `/vision/inference_mode` 切到 `idle`，因此 `/vision/diseased_target_debug_image` 不会持续刷新。这不是识别失败，而是当前流程在喷洒阶段主动停止 YOLO 推理。

## 7. 成功标准

终端日志应依次出现：

```text
MOVING_TO_OBSERVE
SCANNING_TREE
DETECTING_FRUITS
QUEUING
ALIGNING
SPRAYING
RETURNING_TO_OBSERVE
RETURNING_HOME
```

视觉应满足：

- `/vision/tree_debug_image` 能看到 `tree` 框；
- `/vision/diseased_target_debug_image` 能看到 `diseased_target` 标注；
- `/vision/target` 中 `valid: true`；
- `/vision/visual_servo_debug` 最终 `result_code=0`。

喷洒应满足：

- 日志出现 `[SPRAY] service mode, relay service=/relay/set channel=2`；
- 继电器节点日志出现“第 2 路继电器已吸合”，并在时长到期后自动断开；
- `/spray/simulated_active` 在喷洒期间为 `true`，结束后回到 `false`；
- `arm_spray_once` 退出码为 `0`。

## 8. 常见失败与处理

### 8.1 YOLO 不推理

检查：

```bash
ros2 topic echo /mission/status --once
ros2 topic echo /vision/inference_mode
```

原因通常是没有运行 `arm_spray_once`。YOLO 节点只有收到 `/mission/status` 中 `ARM_SPRAYING` 的当前树 ID 后才会工作。

### 8.2 C10 没有图像

检查：

```bash
ros2 topic hz /camera/color/image_raw
ls -l /dev/v4l/by-id/
```

如果设备路径变化，重新启动：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  c10_device:=/dev/v4l/by-id/<现场实际设备名>
```

### 8.3 找不到模型或类别不匹配

检查权重是否安装：

```bash
ls -l ~/WVCSC_S2Z_UTB_ARM/src/wvcsc_rgb_vision/models/yolov8s_real.pt
ls -l ~/WVCSC_S2Z_UTB_ARM/src/wvcsc_rgb_vision/models/yolov8s_seg_real.pt
```

放入新权重后重新构建：

```bash
cd ~/WVCSC_S2Z_UTB_ARM
colcon build --symlink-install --packages-select wvcsc_rgb_vision
source install/setup.bash
```

### 8.4 观察位姿规划失败

优先调整树的相对位置：

- 保持 `tree-x-m` 接近 `0.0`；
- 将 `tree-y-m` 在 `1.30`、`1.40`、`1.50`、`1.60` 或对应负值中逐步尝试；
- 确保玉米树主体在 C10 初始观察视野内；
- 不要先放宽碰撞、奇异或关节限位参数。

### 8.5 对准失败

记录以下话题输出：

```bash
ros2 topic echo /vision/target
ros2 topic echo /vision/visual_servo_debug
ros2 topic echo /servo_node/status
ros2 topic echo /joint_states
```

重点看：

- `target_valid` 是否持续为 `true`；
- `target_age_sec` 是否过大；
- `error_u_px`、`error_v_px` 是否接近但不收敛；
- `servo_status_text` 是否出现 singularity 或 collision；
- `joint_positions` 是否接近关节限位。

## 9. 测试结束

先终止 `arm_spray_once`，再终止 launch。

如需锁定机械臂：

```bash
ros2 topic pub --once /motion_control/command std_msgs/msg/String "{data: stop}"
```

如需回 HOME：

```bash
ros2 topic pub --once /motion_control/command std_msgs/msg/String "{data: reset}"
```

恢复：

```bash
ros2 topic pub --once /motion_control/command std_msgs/msg/String "{data: resume}"
```

## 10. 真实喷洒硬件与继电器控制

当前实机单独测试已经接入真实继电器：

```text
/arm/execute_spray
    → wvcsc_arm_task/spray_actuator
    → /relay/set (wvcsc_interfaces/srv/SetRelay)
    → controller_pkg Modbus RTU
    → 第 2 路继电器
```

喷洒 Action 开始时，`spray_actuator` 会先等待 `/relay/set` 返回
`success=true`，再请求 `channel=2, enabled=true` 并携带喷洒时长。继电器服务端会按
该时长自动断开；Action 完成、取消、运动锁定和节点退出时还会显式发送第 2 路断开。

如果 `/relay/set` 不可用、串口配置错误或继电器返回失败，喷洒 Action 必须失败，不会
把定时器模拟结果误报为真实喷洒成功。仿真环境仍使用 `timer` 模式，不启动
`controller_pkg`。

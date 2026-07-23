# WVCSC_S2Z_UTB_ARM 机械臂单独喷洒流程解耦测试指南

本文档用于现场单独验证机械臂喷洒闭环，不启动小车导航链。

适用目标：

- 小车停在现场固定位置，底盘不运动；
- 参照仿真，将玉米树放在机械臂左侧；
- 只测试 Alicia-M、C10、真实 YOLO、VisualServo、SprayTask 和喷洒 Action；
- 默认使用 `spray_actuator` 的 `timer` 模式验证流程，不直接打开真实喷头或水泵。

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
ls -l "$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/c10_handeye.yaml"
ls -l "$HOME/.ros/wvcsc_calibration/nozzle.yaml"
```

如果喷嘴标定还没有完成，可以先复制示例文件做干流程验证，但不能代表真实落点准确：

```bash
mkdir -p "$HOME/.ros/wvcsc_calibration"
cp "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/nozzle.example.yaml" \
  "$HOME/.ros/wvcsc_calibration/nozzle.yaml"
```

确认 C10 设备：

```bash
ls -l /dev/v4l/by-id/
```

默认设备为：

```text
/dev/video0
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
  c10_device:=/dev/video0 \
  serial_port:=/dev/ttyACM0 \
  arm_velocity_scaling:=0.20 \
  arm_acceleration_scaling:=0.20 \
  use_moveit_rviz:=true
```

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

- `/spray/simulated_active` 在喷洒期间为 `true`；
- 喷洒结束后回到 `false`；
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

## 10. 真实喷洒硬件接入边界

当前单独测试默认使用 `spray_actuator` 的 `timer` 模式，不会控制真实泵/阀。

后续接入真实喷洒硬件时，只替换 `/spray/execute` 的 `wvcsc_interfaces/action/Spray` Action server。上层 `spray_task`、VisualServo、YOLO 和 `arm_spray_once` 不需要改接口。

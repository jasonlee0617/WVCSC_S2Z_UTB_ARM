# C10 眼在手手眼标定指南

适用：ROS 2 Humble、车顶 Alicia-M、C10 RGB、Gazebo Classic 11 与实机。标定类型固定为：

```text
tool0 -> camera_color_optical_frame
```

## 1. 标定环境与固定约定

默认仿真与实机均以完整小车、车顶机械臂和 C10 的统一 URDF 为准。MoveIt 会保留车体碰撞几何，因此仿真中通过的观察姿态可迁移到实机验证。

标定码相对机械臂基座 `alicia_base_link` 安装在左侧：

```yaml
# 实机：wvcsc_calibration/config/auto_handeye_alicia.yaml
marker_position_base_m: [0.0, 0.25, 0.0]

# 仿真：wvcsc_calibration/config/auto_handeye_alicia_sim.yaml
marker_position_base_m: [0.0, 0.25, 0.002]
```

Gazebo 中 marker 由启动脚本相对
`wvcsc_calibration_vehicle::alicia_base_link` 生成。不要再修改 marker 的世界绝对坐标或固定关节角。

旧的“桌子 + 单独 Alicia-M”环境仍保留在：

```text
wvcsc_simulation/worlds/calibration_table.world
wvcsc_calibration/xacro/calibration_arm_camera.urdf.xacro
```

其启动代码以 `LEGACY DESK CALIBRATION ENVIRONMENT` 注释块保留在 `calibration_sim.launch.py`。它缺少车顶安装和车体碰撞模型，只用于历史对比或人工回退；默认 launch 不会启动它，也不能和整车环境同时启动。

## 2. 仿真自动标定

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wvcsc_simulation calibration_sim.launch.py
```

该命令自动启动 Gazebo、整车模型、marker、控制器、MoveIt、C10、ArUco、ArUco 可视化、marker TF、easy_handeye2 和采集器。仿真配置 `auto_start: true`，不需要第二个终端按键。

启动顺序为：

```text
整车生成 -> marker 相对机械臂基座生成 -> Gazebo 解除暂停
-> joint_state_broadcaster -> arm_controller -> gripper_controller
-> MoveIt / ArUco / easy_handeye2 -> 自动采集
```

采集器先根据 marker 先验生成一组更高、更远、更偏移的初始 anchor 候选，再依次执行碰撞 IK、Jacobian 条件数、关节余量和 OMPL 门控。它不再使用固定关节角作为初始观察位。

仿真默认输出：

```text
$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/c10_handeye_sim.yaml
```

调试画面：

```bash
rqt_image_view /calibration/aruco_debug_image
```

在 `rqt_image_view` 中查看该话题时使用 `raw` 传输。不要选择
`compressedDepth`；`/calibration/aruco_debug_image` 是 RGB/BGR 图像，
compressedDepth 只适用于深度图。若终端出现
`compressed_depth_image_transport` 提示 RGB 图像不能压缩为深度图，这是
rqt 传输方式选择噪声，不是 ArUco 检测或手眼标定算法失败。

22 个样本、三算法共识、真值平移范数 `<= 4 mm`、X/Y 单轴误差 `<= 2 mm`、旋转误差 `<= 1 deg` 均通过后才允许进入实机标定。

## 3. 实机自动标定

实机标定只启动机械臂和相机最小链路：Alicia-M 驱动、MoveIt、统一 TF、C10、ArUco、ArUco 可视化、marker TF、easy_handeye2、motion_control 和采集器。

不启动底盘 CAN、底盘驱动、LiDAR、IMU、EKF、Nav2 或 MissionManager。车辆必须停稳、制动并禁止任何 `/cmd_vel` 输入。

终端一：

```bash
ros2 launch wvcsc_calibration c10_handeye.launch.py
```

终端二：

```bash
ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/auto_handeye_alicia.yaml"
```

操作者确认机械臂工作区安全后，在终端二按 `s` 或 Enter 开始；按 `q` 取消。可选安全终端：

```bash
ros2 run wvcsc_arm_task motion_control_keyboard
```

实机默认输出：

```text
$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/c10_handeye.yaml
```

输出文件可纳入版本控制，但只应提交经过现场复测确认的安装外参。

## 4. 速度、加速度与 marker 位置调整

默认缩放：

| 环境 | velocity_scaling | acceleration_scaling |
| --- | ---: | ---: |
| 仿真 | 0.20 | 0.20 |
| 实机 | 0.10 | 0.10 |

启动前可覆盖，例如：

```bash
ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/auto_handeye_alicia.yaml" \
  -p velocity_scaling:=0.15 \
  -p acceleration_scaling:=0.15
```

速度和加速度只在新轨迹规划时读取，不会在执行中的轨迹上动态改变。

若标定码只沿机械臂基座 X/Y 方向移动：

1. 测量 marker 中心相对 `alicia_base_link` 的 X/Y。
2. 修改实机 `marker_position_base_m`；仿真同时修改 overlay 中对应值。
3. 不修改 `initial_joint_positions`（该参数已移除）、候选姿态源码或 Gazebo 世界坐标。
4. 重新运行仿真真值验证，再进行实机复标。

## 5. C10 内参与 Gazebo 限制

实机和仿真均以以下文件为唯一内参来源：

```text
package://wvcsc_c10_camera/config/c10_intrinsics.yaml
```

Gazebo 渲染使用其中的 `fx`、`fy`、`cx`、`cy` 与畸变参数。Gazebo Classic 的 `gazebo_ros_camera` 发布的 `CameraInfo.K` 只能使用单一焦距，因此 K 中 `fy` 近似为 `fx`；`P_fy` 仍使用真实 fy。该限制已在仿真真值门控中保留，不额外引入 CameraInfo 中继节点。

## 6. 运行前检查

```bash
ros2 control list_controllers
ros2 topic hz /camera/color/image_raw
ros2 topic echo /calibration/aruco_debug_image --once
ros2 topic echo /aruco_markers --once
ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame
ros2 run tf2_ros tf2_echo camera_color_optical_frame calibration_aruco
```

控制器必须为 `active`，marker 必须稳定可见；若没有安全初始观察位、marker 不可见或质量门控失败，采集器不会降低碰撞、奇异性或关节余量阈值。

`handeye_server` 启动早期如果提示 `calibration_aruco` TF 暂不可用，
通常是可恢复状态：机械臂移动到初始 anchor 并且相机稳定看到标定码后，
marker TF 会发布，随后出现 `All expected transforms are available` 才表示
easy_handeye2 采样服务真正可用。只有长时间无法出现该 ready 日志，才应继续
检查相机图像、ArUco 可视化、marker 位置先验和 TF 链路。

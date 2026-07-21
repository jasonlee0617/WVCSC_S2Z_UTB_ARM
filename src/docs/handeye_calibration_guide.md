# C10 眼在手手眼标定指南

适用：ROS 2 Humble、Alicia-M、C10 RGB、Gazebo Classic 11 与实机。标定类型固定为眼在手：`tool0 → camera_color_optical_frame`。

## 仿真自动标定

终端一启动完整标定环境：

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wvcsc_simulation calibration_sim.launch.py
```

该 launch 使用 Gazebo Classic 的控制器启动链：

```text
spawn → unpause Gazebo (zero gravity) → joint_state_broadcaster
      → arm_controller → gripper_controller
      → motion_control + ArUco + marker TF + handeye server
```

控制器任一环节失败会停止 launch，不会继续执行采集。标定场景将重力固定为零：这使控制器在解除暂停后立即获得更新周期，同时避免未激活机械臂因重力跌落；该设置仅用于几何标定，不代表真实动力学。Gazebo场景中桌面为 `1.20 × 0.80 m`，四条桌腿可见；仿真和实机均使用70 mm编码区域、90 mm背板的ArUco码，位于桌面 `(0.45, 0, 0.752)`，朝上。仿真为严格3 mm外参基准使用无噪声图像和68.8度水平视场，使真实尺寸的平面码获得足够像素；这个相机FOV仅服务于几何基准，不替代实机内参标定。

终端二运行与实机共用的采集器：

```bash
ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/auto_handeye_alicia.yaml" \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/auto_handeye_alicia_sim.yaml"
```

按键：

```text
s / Enter  开始新的自动采集会话
q          取消当前会话并返回开始前关节姿态
Ctrl+C     取消并退出
```

可选终端三用于安全介入：

```bash
ros2 run wvcsc_arm_task motion_control_keyboard
```

`SPACE` stop并锁定，`h` reset并HOME，`r` resume。标定过程发生stop后，该轮会话作废，必须重新按`s`开始。

采集器先进入Alicia官方参考姿态：

```text
[0.0, -1.09, -0.87, 0.0, -0.77, 0.0]
```

之后围绕实时识别的标定码生成21个候选姿态。每个候选均通过碰撞IK、雅可比条件数、关节余量、OMPL规划和RGB图像质量检查；目标为18个有效样本，最低15个。若21个广域候选的安全子集不足目标样本数，采集器先加入8个水平`yaw+roll`宽激励和8个`10°–12°`离轴视角，再加入12个`±3°/±15 mm`细粒度候选；总候选最多49个，仍逐个通过相同门控。宽激励和离轴视角避免水平桌面码下径向位移、纯roll和始终居中观察造成的旋转退化；细粒度候选只用于补足可达样本。标定期间允许标记偏离图像中心最多220 px，但仍要求角点边缘余量和稳定性，离轴姿态不会被重心步骤拉回中心。若官方参考姿态可见标定码但接近腕部奇异位形，采集器最多三次先移动到已通过全部门控的安全观察位，再围绕该位重新生成候选；重复锚定时会避开不改变姿态的`seed`候选，不会降低条件数或关节余量阈值。仿真额外向MoveIt添加机械臂前方桌面碰撞盒，避免候选从桌面下方穿越。仿真相机图像噪声固定为零，仅作为URDF外参的确定性几何基准；实机仍必须依赖RGB质量门控和重复标定。

采集仍由 easy_handeye2 服务保存原始样本；WVCSC 求解器独立使用 ROS 的
`base→tool0`、`camera→marker` 正向样本约定，并统一导出
`tool0→camera_color_optical_frame`。这样仿真真值和实机输出使用同一条可测试的
OpenCV求解链，而不依赖第三方服务端的保存格式。

仿真输出：

```text
~/.ros/wvcsc_calibration/c10_handeye_sim.yaml
```

仿真会将求解外参与URDF中的已知 `tool0 → camera_color_optical_frame` 外参比较。硬门槛为平移不超过`3 mm`、旋转不超过`1°`；超限不会保存或覆盖输出文件。

```text
[CALIBRATION][GROUND_TRUTH] translation_error=...mm rotation_error=...deg
```

2026-07-22 已在 Gazebo Classic 11 完成一次完整回归：22 个有效样本，Park
解的真值误差为 **2.72 mm / 0.38°**，三算法最大分歧为 `0.20 mm / 0.19°`，
固定码残差 RMS 为 `0.47 mm / 0.43°`。这是无图像噪声、固定桌面标记条件下
对 URDF 外参的基准结果；不代表真实 C10 或实际安装后的标定精度。

## 实机自动标定

将70 mm的编码区域打印在硬质90 mm背板上，固定在平整桌面且采样期间不可移动。每次 `take_sample` 前，采集器使用角点亚像素化后的质量合格 RGB 观测计算稳定 `camera→marker` 位姿，并临时发布给唯一的 `marker_tf` TF authority；原始 ArUco 话题仍用于实时就绪检查。这样不会在机械臂移动过程中取样，也不会产生第二个 TF 发布者。启动：

```bash
ros2 launch wvcsc_calibration c10_handeye.launch.py

ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/auto_handeye_alicia.yaml"
```

实机同样使用RGB ArUco与CameraInfo估计六自由度marker位姿，不需要深度图。实机默认关闭仿真真值门槛；保留15个最小样本、18个目标样本、算法一致性和固定marker残差门槛。

如果实机桌面位于机械臂工作区内，应在启动采集器前用参数配置其相对 `alicia_base_link` 的位置和尺寸，并启用：

```yaml
calibration_surface_enabled: true
```

最终实机部署文件：

```text
~/.ros/wvcsc_calibration/c10_handeye.yaml
```

## 运行前检查

```bash
ros2 control list_controllers
ros2 topic hz /camera/color/image_raw
ros2 topic echo /aruco_markers --once
ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame
ros2 run tf2_ros tf2_echo camera_color_optical_frame calibration_aruco
```

所有控制器必须为`active`，ArUco码必须稳定可见后才按`s`。

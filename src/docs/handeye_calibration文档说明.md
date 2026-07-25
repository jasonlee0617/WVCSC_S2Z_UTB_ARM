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

采集器先根据 marker 先验生成已验证可达的初始 anchor 候选，再依次执行碰撞 IK、Jacobian 条件数、关节余量和 OMPL 门控。它不使用固定关节角作为初始观察位，也不会降低安全阈值来换取可见性。

仿真默认输出为 config 目录下同一时间戳的一对文件：

```text
c10_handeye_sim_YYYYMMDD_HHMMSS.calib
c10_handeye_sim_YYYYMMDD_HHMMSS.yaml
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

至少 18 个有效样本、目标 22 个样本、三算法共识、真值平移范数 `<= 3 mm`、X/Y 单轴误差 `<= 2 mm`、旋转误差 `<= 1 deg` 均通过后才允许写出仿真标定结果。候选可达性只要求满足 18 个样本下限，不会因为未预先找到完整 22 个安全候选而拒绝启动采样；每个视角族仍会预规划当前配置的前 3 个安全候选。若需安全恢复换位，已通过安全门控的候选会合并为同一采样池，为图像质量拒绝保留冗余；每次执行前仍重新核验/规划，不复用失效轨迹。采样阶段会尽可能收集到 22 个。真值只用于仿真验收，不参与样本选择或求解。

OpenCV 的 Park/Horaud/Tsai-Lenz 闭式解首先提供确定性初值；随后默认启用固定标定码一致性细化。它将每条样本的 `base -> tool0 -> camera -> marker` 反投影到同一个未知的 `base -> marker`，最小化这些测得位姿之间的平移和旋转残差。这一步只使用样本本身、不会读取仿真外参真值，也不替代三算法共识、异常样本剔除或最终 `3 mm / XY 2 mm / 1 deg` 门控。仿真使用 `0.25 mm / 1 deg` 的测量权重；实机默认权重更保守，若需调整必须保留现场复测记录。

每一条样本还必须满足**末端真实静止**：`arm_controller` 以
`allow_nonzero_velocity_at_trajectory_end: false` 结束轨迹；采集器在既有
`settle_time_sec` 后，继续要求连续 `joint_stationary_window_sec`（仿真为
`0.30 s`）内六关节位置最大跨度不超过
`joint_stationary_max_position_delta_rad`（仿真为 `0.0001 rad`）。这一步直接验证
`tool0` TF 已收敛，防止“相机已经取到 PnP 图像、机械臂位置仍在变化”造成错时配对；
超时只会拒绝当前候选，绝不会降低 3 mm / 2 mm / 1 deg 验收门槛。Gazebo 当前的
`/joint_states.velocity` 与相邻位置差分不一致，因此它只记录为诊断，不作为通过
判据；实机仍应根据位置测量噪声做有记录的调整，而不能为通过一次采样而盲目放宽。

仿真配置允许用 `acquisition_corner_margin_px: 12` 先获取仍完整可见、但靠近画面边缘的标定码，再使用原有碰撞检查和最多 `3 × 6 mm` 的图像平面重心校正将其移回主视野。`maximum_center_error_px: 130` 只决定何时值得执行这一步校正；它不是样本质量或精度门槛。注意：若码的角点距画面边缘仍小于 `minimum_corner_margin_px: 60`，即使中心偏差小于 130 px，也必须继续校正，不能绕过严格边缘门控。车载安全锚点的实测 PnP 距离稳定在约 `0.250 m`；原 `0.25 m` 下限会因亚像素 PnP 的浮点扰动误拒绝同一物理姿态，因此仅仿真配置使用 `marker_distance_min_m: 0.24`。这不是对样本精度的放弃：每次写入样本前仍重新执行 `minimum_corner_margin_px: 60`、`minimum_marker_side_px: 90`、距离、角点尺度、平面稳定度和姿态稳定度的严格检查；仿真真值 `3 mm / XY 2 mm / 1 deg` 门控不变。实机仍保持 `marker_distance_min_m: 0.25`，除非现场数据复现相同边界问题并经精度验收后再调整。

仿真还设置 `minimum_safe_candidates: 30`：这是安全轨迹的储备数量，不是可写入样本数。这样当少数候选在最终图像门控、重心校正或多样性检查中被拒绝时，采集器会换到另一安全锚点继续生成候选，并继续尝试剩余的预筛安全轨迹；`maximum_samples` 限制的是已写入样本数，而不是尝试次数。流程仍坚持 `minimum_samples: 18` 和上述全部严格质量、真值门控。实机保持默认候选储备，需根据现场成功率单独评估后再修改。

固定标定码一致性细化使用 `fixed_marker_refinement_translation_sigma_m` 与 `fixed_marker_refinement_rotation_sigma_deg` 表示观测残差的尺度，而不是精度门槛。Gazebo 当前实测样本的固定码 RMS 约为 `0.38 mm / 0.31 deg`，因此仿真采用 `0.50 mm / 0.30 deg`；它避免把平移残差过度加权、同时保留可靠的姿态约束。该细化只使用每个样本都应满足的 `base→tool→camera→marker = 固定 base→marker` 关系，绝不读取 `ground_truth_*` 参数参与解算；真值仅在结果后用于 `3 mm / XY 2 mm / 1 deg` 的验收。

仿真还启用 `use_marker_position_prior_for_candidate_generation: true`。该位置与 `calibration_sim.launch.py` 的 Gazebo 标定码生成位置来自同一 `marker_position_base_m`，只用来在机械臂运动过程中稳定地生成候选视角，避免由延迟 PnP TF 重新规划到画面边缘。它不写入 easy_handeye2 样本、不参与 OpenCV 求解，也不替代最终图像 PnP 质量检查；实机保持 `false`，始终使用测得的 `base -> calibration_aruco` TF。仿真候选只允许 `camera_centering_scale_candidates >= 0.25`：不能为增加 IK 数量退回 `0.0`，否则 C10 的偏置主点会让标定码系统性偏离图像几何中心约 165 px。

图像重心校正的目标统一为图像几何中心 `(width/2, height/2)`，而不是相机内参主点 `(cx, cy)`。这是因为候选视角生成已使用 C10 的 `cx/cy` 偏移补偿，使标定码落在几何中心；两个阶段若混用目标点，会把正确画面再向下移动约 165 px，导致严格边缘门控错误拒绝。

## 3. 实机自动标定

实机标定分为两个终端：终端 A 启动机械臂和相机最小链路（Alicia-M 驱动、MoveIt、统一 TF、C10、ArUco、ArUco 可视化、marker TF、easy_handeye2、motion_control）；终端 B 启动需要人工确认的采集器。

不启动底盘 CAN、底盘驱动、LiDAR、IMU、EKF、Nav2 或 MissionManager。车辆必须停稳、制动并禁止任何 `/cmd_vel` 输入。

两个终端都必须隔离用户目录 Python 包。系统 `transforms3d` 仍使用
`np.float`，若 `/home/eisa/.local` 的新版 NumPy 被导入，ArUco 与
easy_handeye2 会直接崩溃。先在终端 A 执行：

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
import numpy
import transforms3d
print(numpy.__file__)
print(transforms3d.__file__)
PY

ros2 launch wvcsc_calibration auto_handeye.launch.py \
  video_device:=/dev/video2 serial_port:=/dev/ttyACM0
```

上面的 Python 检查中，NumPy 路径不得位于 `/home/eisa/.local`。终端 A
只编排 C10、Alicia-M、MoveIt、ArUco、marker TF 与 easy_handeye2；它不会
启动采集器，因为 `ros2 launch` 的子进程没有交互 TTY，无法安全接收 `s/Enter`。

在所有服务和图像就绪后，另开终端 B，执行相同的环境隔离并启动采集器：

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/auto_handeye_alicia.yaml"
```

终端 B 中由操作者按 `s` 或 Enter 才开始运动；按 `q` 取消。可选安全终端：

```bash
ros2 run wvcsc_arm_task motion_control_keyboard
```

仅当采样、PnP 质量、多算法共识和固定标定码残差均通过后，实机默认在 source config
目录原子写入两份同一变换。文件名使用同一时间戳：

```text
c10_handeye_YYYYMMDD_HHMMSS.calib
c10_handeye_YYYYMMDD_HHMMSS.yaml
```

实机启动入口自动选择最新的 `.calib` 文件，第二份为 WVCSC 归一化部署配置。任一质量门控或
写入预处理失败都不会覆盖已有标定。输出文件可纳入版本控制，但只应提交经过现场复测确认的安装外参。

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

当前 C10 已切换为 OST 导出的真实 640×480 内参，驱动、Gazebo、ArUco 和
喷嘴投影均必须继续使用同一分辨率。更换分辨率或裁剪模式后必须重新标定，
不能继续复用当前 K/D 直接宣称毫米级几何精度。

Gazebo 渲染使用其中的 `fx`、`fy`、`cx`、`cy` 与畸变参数。Gazebo Classic 的 `gazebo_ros_camera` 发布的 `CameraInfo.K` 只能使用单一焦距，因此 K 中 `fy` 近似为 `fx`；`P_fy` 仍使用真实 fy。该限制已在仿真真值门控中保留，不额外引入 CameraInfo 中继节点。

仿真 ArUco 模型的黑白单元以 `20 um` 的最小渲染偏移贴合白色底板，避免 z-fighting；不得恢复为毫米级凸起方块。后者会在斜视角形成侧壁和阴影，使角点位置出现系统性偏差，并直接损害手眼标定的旋转精度。

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

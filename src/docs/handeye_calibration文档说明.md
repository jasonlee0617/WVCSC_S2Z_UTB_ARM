# C10 眼在手手眼标定指南

适用：ROS 2 Humble、车顶 Alicia-M、C10 RGB、Gazebo Classic 11 与实机。标定类型固定为：

```text
tool0 -> camera_color_optical_frame
```

## 1. 标定环境与固定约定

默认仿真与实机均以完整小车、车顶机械臂和 C10 的统一 URDF 为准。MoveIt 会保留车体碰撞几何，因此仿真中通过的观察姿态可迁移到实机验证。

仿真启动文件以 `marker_position_base_m: [0.595, -0.030, 0.002]` 相对
`alicia_base_link` 生成桌面上的标定码。该点按官方20个固定关节姿态、当前Alicia-M
URDF和C10内参做投影覆盖率预检后选定：19个姿态满足严格的边缘余量、标记尺寸和距离
门控，因此可以保留14个有效样本的下限，而不降低图像质量标准或重复固定姿态表。标定码
表面位于 `z=2 mm`，启动文件以水平、朝上的姿态生成它。实机应按同一覆盖率原则固定
同尺寸标定码；该位置不用于生成采样姿态。

Gazebo 中 marker 由启动脚本相对
`wvcsc_calibration_vehicle::alicia_base_link` 生成。不要再修改 marker 的世界绝对坐标或固定关节角。

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

采集器严格按 Alicia-M 官方 20 个固定关节位姿和既定顺序采样。每一行仍会先检查 Jacobian 条件数与关节余量，再通过 `AliciaMoveIt.move_joints()` 的 MoveIt/OMPL 碰撞规划执行；停稳、C10 亚像素 ArUco 质量、TF 和多样性门控任一失败都会记录并跳过该点，绝不降低安全阈值或使用直接控制器回退。

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

仿真必须在固定表中取得至少 14 个有效样本，并通过三算法共识、真值平移范数 `<= 3 mm`、X/Y 单轴误差 `<= 2 mm`、旋转误差 `<= 1 deg` 后才允许写出标定结果。达到下限即停止遍历，若全部 20 点结束仍不足下限则失败且不保存；真值只用于最终仿真验收，不参与样本选择或求解。

OpenCV 的 Park/Horaud/Tsai-Lenz 闭式解首先提供确定性初值；随后默认启用固定标定码一致性细化。它将每条样本的 `base -> tool0 -> camera -> marker` 反投影到同一个未知的 `base -> marker`，最小化这些测得位姿之间的平移和旋转残差。这一步只使用样本本身、不会读取仿真外参真值，也不替代三算法共识、异常样本剔除或最终 `3 mm / XY 2 mm / 1 deg` 门控。仿真使用 `0.25 mm / 1 deg` 的测量权重；实机默认权重更保守，若需调整必须保留现场复测记录。

每一条样本还必须满足**末端真实静止**：`arm_controller` 以
`allow_nonzero_velocity_at_trajectory_end: false` 结束轨迹；采集器在既有
`settle_time_sec` 后，继续要求连续 `joint_stationary_window_sec`（仿真为
`0.30 s`）内六关节位置最大跨度不超过
`joint_stationary_max_position_delta_rad`（仿真为 `0.0001 rad`）。这一步直接验证
`tool0` TF 已收敛，防止“相机已经取到 PnP 图像、机械臂位置仍在变化”造成错时配对；
超时只会拒绝当前固定关节点，绝不会降低 3 mm / 2 mm / 1 deg 验收门槛。Gazebo 当前的
`/joint_states.velocity` 与相邻位置差分不一致，因此它只记录为诊断，不作为通过
判据；实机仍应根据位置测量噪声做有记录的调整，而不能为通过一次采样而盲目放宽。

固定序列不做图像重心候选生成或相机外参 bootstrap。每次写入样本前仍严格执行 `minimum_corner_margin_px: 60`、`minimum_marker_side_px: 90`、距离、角点尺度、平面稳定度和姿态稳定度门控；仿真真值 `3 mm / XY 2 mm / 1 deg` 门控不变。实机保持 `marker_distance_min_m: 0.25`，现场改变标定码位置后必须先完成可见性验证。

固定标定码一致性细化使用 `fixed_marker_refinement_translation_sigma_m` 与 `fixed_marker_refinement_rotation_sigma_deg` 表示观测残差的尺度，而不是精度门槛。固定关节表的 C10 回归将固定码位置/旋转 RMS 一致性门槛设置为 `2.0 mm / 0.60 deg`：前者用于剔除相互矛盾的样本，后者允许平面 PnP 的可测角度离散；它们不是外参精度声明。细化仍采用 `0.50 mm / 0.30 deg` 残差尺度，且只使用每个样本都应满足的 `base→tool→camera→marker = 固定 base→marker` 关系，绝不读取 `ground_truth_*` 参数参与解算；真值仅在结果后用于 `3 mm / XY 2 mm / 1 deg` 验收。

标定结果仍完全来自测得的 `base -> tool0` 与 `camera -> calibration_aruco` 样本；Gazebo 的已知外参只在最终验收阶段比较，绝不参与运动目标生成或求解。

## 3. 实机自动标定

实机标定只启动机械臂和相机最小链路：Alicia-M 驱动、MoveIt、统一 TF、C10、ArUco、ArUco 可视化、marker TF、easy_handeye2、motion_control 和采集器。

不启动底盘 CAN、底盘驱动、LiDAR、IMU、EKF、Nav2 或 MissionManager。车辆必须停稳、制动并禁止任何 `/cmd_vel` 输入。

启动一条完整的最小链路：

```bash
ros2 launch wvcsc_calibration auto_handeye.launch.py \
  video_device:=/dev/video0 serial_port:=/dev/ttyACM0
```

该入口只编排 C10、Alicia-M、MoveIt、ArUco、marker TF、easy_handeye2 与
现有的安全采集器；不另建 IK、轨迹或直接关节控制路径。所有服务和图像
就绪后，采集器仍等待操作者按 `s` 或 Enter 才开始运动；按 `q` 取消。可选安全终端：

```bash
ros2 run wvcsc_arm_task motion_control_keyboard
```

仅当采样、PnP 质量、多算法共识和固定标定码残差均通过后，实机默认原子写入两份同一变换：

```text
~/.ros2/easy_handeye2/calibrations/wvcsc_c10.calib
$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/c10_handeye.yaml
```

第一份供实机启动入口读取，第二份供 WVCSC 部署配置读取。任一质量门控或
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

1. 按官方固定关节序列对应的桌面位置固定 marker，并测量其相对 `alicia_base_link` 的 X/Y。
2. 仿真同时修改 overlay 的 `marker_position_base_m`。
3. 不修改固定关节表、Gazebo 世界绝对坐标或安全门控。
4. 重新运行仿真真值验证，再进行实机复标。

## 5. C10 内参与 Gazebo 限制

实机和仿真均以以下文件为唯一内参来源：

```text
package://wvcsc_c10_camera/config/c10_intrinsics.yaml
```

当前 C10 内参来自 640×480 标定后向 1280×720 的近似映射；由于两个分辨率
并非等比例缩放，当前文件只保证现有图像与 ArUco 链路可工作，**不能据此
宣称实机具有毫米级几何精度**。需要该精度时，应以实际 C10 发布分辨率重新
标定内参；本流程不会把该近似内参当成额外质量门控，也不会自动重标定。

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

控制器必须为 `active`，marker 必须稳定可见；固定点的 MoveIt 规划、marker 可见性或质量门控失败时，采集器只跳过该点，不会降低碰撞、奇异性或关节余量阈值。

`handeye_server` 启动早期如果提示 `calibration_aruco` TF 暂不可用，
应先检查相机图像、ArUco 可视化和 TF 链路；固定序列开始前必须出现
`All expected transforms are available`，否则采集器不会运动。

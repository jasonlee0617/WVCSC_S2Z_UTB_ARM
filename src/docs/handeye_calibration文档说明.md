# C10 眼在手手眼标定指南

标定结果固定描述 `tool0 -> camera_color_optical_frame`。实机与 Gazebo 使用同一套采集器、固定 20 姿态、采样状态机、求解器和 Qt 界面；但启动链路、传感器、控制器、时间源、部分质量参数、结果目录和仿真真值验收不同，并不是只替换 YAML。

## 1. 文件与保存规则

```text
wvcsc_perception/wvcsc_calibration/config/
├── aruco_c10.yaml                  # 实机和仿真共用
├── nozzle.example.yaml              # 实机和仿真共用
├── real/
│   ├── auto_handeye_alicia.yaml
│   ├── c10_handeye_20260727_144847.calib
│   ├── c10_handeye_YYYYMMDD_HHMMSS.calib
│   └── c10_handeye_YYYYMMDD_HHMMSS.samples
└── sim/
    ├── auto_handeye_alicia_sim.yaml
    ├── c10_handeye_sim_YYYYMMDD_HHMMSS.calib
    └── c10_handeye_sim_YYYYMMDD_HHMMSS.samples
```

`c10_handeye_20260727_144847.calib` 是原 YAML 的 native easy_handeye2 转换结果。今后的成功结果只写 native `.calib` 与同一时间戳的 `.samples`，不会再写结果 YAML。

- 完整成功：保存 `.calib` 和 `.samples`。
- 质量、求解或仿真真值门槛失败：只要已有至少一条有效样本，就只保存 `.samples`。
- Qt 点击“复位”、CLI 输入 `q` 或任何取消：不保存文件。
- 常规成功或失败后回到标定初始姿态（READY）；只有“复位”才停止并回 HOME。

`nozzle.example.yaml` 保持在 `config/` 根目录，不随 real/sim 拆分。

## 2. 默认 Qt 模式

Qt 是两个 launch 的默认模式。界面左侧提供启动、采集、复位和采集器终端输出；右侧是一个可收起的图像面板，默认优先显示 `/calibration/aruco_debug_image`，下拉框可切到 `/camera/color/image_raw`。仿真真值误差只写入下方终端日志，不再单独显示在顶部；实机没有仿真真值输出。

- 启动：机械臂执行固定标定初始姿态，不采样。
- 采集：从初始姿态开始执行质量门控采样和求解。
- 复位：先发布 `stop`，收到 `STOPPED_LOCKED` 后再发布 `reset`；只有收到 `RUNNING` 才表示 HOME 完成。
- 若机械臂离开过 HOME，关闭 Qt 会被拦截，必须先完成复位。

### Gazebo Qt

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wvcsc_simulation calibration_sim.launch.py
```

Gazebo Qt 的终端区会输出每条样本质量、求解质量、输出路径，以及与仿真外参真值的平移范数和旋转角误差。只有同时出现以下两条日志才可称为仿真标定通过：

```text
[CALIBRATION][GROUND_TRUTH] ... passed=True
[CALIBRATION] SUCCESS ...
```

当前仿真最终门槛是总平移 `<= 3 mm`、X/Y 分量各 `<= 2 mm`、旋转 `<= 1 deg`。这些门槛不是采样数量本身；质量差的原因可能是图像、PnP、停稳、姿态覆盖、求解一致性或真值门槛中的任一个。

### 实机与仿真的共同流程和差异

两种模式共用以下标定流程：准备标定初始位姿 → 按固定 20 个关节姿态运动 → 等待关节停稳 → 检查 ArUco 图像稳定性和视野质量 → 调用 easy_handeye2 记录样本 → 进行固定标记优化和离群样本处理 → 检查样本覆盖与求解质量 → 保存 `.calib` 和 `.samples`。

差异来自运行环境和配置，而不只是 YAML 文件名：

- Gazebo 启动世界、机器人和标定码模型，并使用仿真相机、Gazebo 控制器与 `use_sim_time=true`；仿真使用 P 投影内参，并在求解后执行外参真值校验。
- 实机启动真实 C10 相机、真实机械臂串口和实机控制链路，使用 K 内参、`use_sim_time=false`，不生成 Gazebo 标定码，也不执行仿真真值校验。
- 两种模式的运动速度、停稳位置窗口、质量门槛、固定标记优化权重、输出目录和文件前缀可以不同；动作顺序和采样状态机保持一致。

### 实机 Qt

实机启动前车辆必须停稳、制动，且不得同时运行完整任务/MissionManager。C10 与 Alicia-M 默认设备是 `/dev/video2` 和 `/dev/ttyACM0`，其他机器请显式覆盖。

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wvcsc_calibration auto_handeye.launch.py \
  video_device:=/dev/video2 serial_port:=/dev/ttyACM0
```

`unset PYTHONPATH` 必须在两次 `source` 之前执行；它避免用户目录的新版 NumPy 破坏 ROS Humble 的 `transforms3d`。

实机没有 Gazebo 真值，因此 Qt 只报告样本/求解质量，不能把 `marker_rms` 当成绝对外参误差。绝对误差必须用独立夹具或独立观测验证。

## 3. CLI 模式（无 Qt）

CLI 与 Qt 是替代方案。一个运行中的 Qt 已内嵌采集器；此时绝不能再启动第二个 `auto_calibration_collector`，否则两个进程会争用同一机械臂和 easy_handeye2 样本列表。

### Gazebo CLI

终端 A：只启动 Gazebo 标定基础链路，不启动 Qt/采集器。

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wvcsc_simulation calibration_sim.launch.py use_calibration_qt:=false
```

终端 B：启动唯一的交互采集器。两个参数文件都必须传入：real 文件提供共用采样参数，sim 文件覆盖仿真参数和 `config/sim` 输出目录。

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/real/auto_handeye_alicia.yaml" \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/sim/auto_handeye_alicia_sim.yaml"
```

在终端 B 输入 `s` 或直接按 Enter：执行“初始位姿 -> 采集”组合流程。输入 `q` 取消且不保存。

若希望分步控制，也可在第三个终端调用：

```bash
ros2 service call /calibration/prepare std_srvs/srv/Trigger "{}"
ros2 topic echo /calibration/state --once
ros2 service call /calibration/collect std_srvs/srv/Trigger "{}"
```

### 实机 CLI

终端 A：启动基础链路但关闭 Qt。

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wvcsc_calibration auto_handeye.launch.py \
  use_calibration_qt:=false video_device:=/dev/video2 serial_port:=/dev/ttyACM0
```

终端 B：同样隔离 Python 后启动唯一采集器。

```bash
cd "$HOME/WVCSC_S2Z_UTB_ARM"
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file "$(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/real/auto_handeye_alicia.yaml"
```

输入 `s`/Enter 或使用上节的 `/calibration/prepare`、`/calibration/collect` 服务。`q` 取消且不保存。

任何模式的紧急复位都应等状态机确认，而不是使用固定延时：

```bash
ros2 topic pub --once /motion_control/command std_msgs/msg/String "{data: stop}"
ros2 topic echo /motion_control/state --once
# 确认上一条输出为 STOPPED_LOCKED 后：
ros2 topic pub --once /motion_control/command std_msgs/msg/String "{data: reset}"
ros2 topic echo /motion_control/state --once
```

最后一条必须为 `RUNNING` 才表示 HOME 成功；`RESET_FAILED` 时保持锁定并排查 MoveIt/碰撞/硬件，不要强行重新采集。

## 4. 采样和故障定位

固定 20 姿态每一条都要通过 MoveIt 碰撞规划、Jacobian 条件数、关节余量、关节停稳、ArUco 画面质量、姿态多样性和求解质量。默认至少 15 条有效样本，求解最少保留 14 条；达不到不降低阈值。

运行前可检查：

```bash
ros2 control list_controllers
ros2 topic hz /camera/color/image_raw
ros2 topic echo /calibration/aruco_debug_image --once
ros2 topic echo /aruco_markers --once
ros2 topic echo /calibration/state --once
```

若采样失败，先查看 Qt 或采集器终端中每条 `image_quality`、关节停稳、样本覆盖与 `marker_rms` 日志；不要仅因“样本数不足”就假设门槛过高或直接放宽门槛。

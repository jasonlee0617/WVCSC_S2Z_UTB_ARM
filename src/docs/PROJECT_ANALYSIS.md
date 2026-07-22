# WVCSC_S2Z_UTB_ARM 项目分析

> 更新日期：2026-07-20
> 详细架构：[WORKSPACE_ARCHITECTURE.md](WORKSPACE_ARCHITECTURE.md)
> 仿真执行手册：[WVCSC_S2Z_UTB_ARM_Codex_仿真任务闭环实施方案.md](WVCSC_S2Z_UTB_ARM_Codex_仿真任务闭环实施方案.md)

## 1. 项目定位

智能农林病虫害空地协同防治：无人机提供疑似病树坐标，阿克曼小车自主停靠，Alicia-M 六轴机械臂利用 C10 RGB 相机识别病果并通过 IBVS 图像伺服逐个对准后喷洒。

**仿真闭环已 100% 完成。下一步转入实机部署。**

## 2. 当前闭环

```text
Mock/Replay UAV
  → DiseaseTreeArray
  → Mission Manager 生成 0.2 m 横向停靠位姿
  → Nav2 NavigateToPose
  → /odom 连续停稳 (1.0s)
  → Arm ExecuteSpray
  → ObservationOptimizer 动态观察位姿 (条件数≤16.5, 关节余量≥0.22rad)
  → YOLOv8n Tree Detect (conf=0.10)
  → YOLOv8n-seg Fruit Instance Segmentation (conf=0.20)
  → 病果去重 + TargetRecenter 重心修正
  → 30 Hz IBVS Twist → 100 Hz MoveIt Servo (fine_tolerance=1.5px, stable=0.5s)
  → Spray Action
  → RETURNING_TO_OBSERVE 复检 → HOME → 下一树
```

### 验证指标

| 指标 | 当前值 | 状态 |
|------|--------|------|
| 四树闭环 | 4/4 完成 | 通过 |
| 对准精度 | ≤ 2 px/轴 | 通过 |
| 对准中位时间 | 1.4 s | 通过 |
| 连续三轮 | 待验收 | 未完成 |
| 构建 | 24 包通过 | 通过 |
| 测试 | 196 tests, 0 failures | 通过 |

## 3. 包清单（24 个）

| 层 | 包 | 语言 | 说明 |
|----|-----|------|------|
| 硬件驱动 | `serial`, `can_bridge`, `yesense_interface`, `yesense_std_ros2`, `wtb_car_driver`, `lslidar_driver`, `lslidar_msgs` | C++ | 串口/CAN/IMU/LiDAR/底盘 |
| 机械臂 | `alicia_m_descriptions`, `alicia_m_driver`, `alicia_m_moveit_config`, `alicia_m_bringup`, `alicia_m_calibration` | C++ | URDF/ros2_control/MoveIt/手眼标定 |
| SLAM/导航 | `my_cartographer`, `my_navigation2` | C++ | Cartographer + Nav2 |
| 运动控制 | `pymoveit2`, `trajectory_retime_server` | C++ | Python MoveIt2 封装 + 轨迹重定时 |
| 复合模型 | `wvcsc_description` | C++ | 统一 XACRO（底盘+Alicia+C10+喷嘴）|
| 仿真 | `wvcsc_simulation` | C++ | Gazebo 世界生成、AckermannSim、数据采集 |
| 接口 | `wvcsc_interfaces` | C++ | 自定义 action/srv/msg |
| 任务编排 | `wvcsc_mission_manager`, `wvcsc_uav_gateway` | Python | 使命管理 + Mock/Replay UAV |
| 感知 | `wvcsc_rgb_vision`, `wvcsc_c10_camera` | Python | YOLOv8n 两级推理 + C10 驱动 |
| 执行 | `wvcsc_arm_task`, `wvcsc_visual_servo` | Python | 喷洒状态机 + IBVS 伺服 |

**收敛记录**：
- 删除 `wvcsc_web_ui`（与 Nav2 Qt 重叠）—— 26→25 包
- 删除 7 个无调用方的独立 launch
- `wvcsc_simulation/data_acquisition/` 解耦离线数据采集与运行时代码
- `alicia_m_grasp_6d` 移入 experimental/
- `wvcsc_spray_controller` 保留但默认不构建（真机泵阀接入时启用）

## 4. SprayTask 架构

`SprayTask` 采用 **Mixin 分解模式**，核心逻辑拆分为三个混入类：

```
SprayTask(TargetFlowMixin, ObservationFlowMixin, DownstreamActionMixin, Node)
    │
    ├── TargetFlowMixin        # 视觉目标流：YOLO 检测接收、去重、重心修正、目标生命周期
    │   ├── _on_tree_detections()     # YOLO tree 检测回调
    │   ├── _on_fruit_detections()    # YOLO fruit 检测回调
    │   ├── _on_selected_target()     # IBVS 锁定目标回调
    │   ├── _wait_for_fruits()        # 稳定候选等待
    │   ├── _queue()                  # 去重排序
    │   ├── _recenter_target()        # 重心修正 (48px触发, ≤20°/次, ≤2次迭代)
    │   └── target_validation         # 后重心稳定校验 (0.2s, ≤4px漂移)
    │
    ├── ObservationFlowMixin   # 观察位姿：动态生成、IK 筛选、扇形扫描
    │   ├── _move_to_observation()     # TF变换 + camera_look_at_pose
    │   ├── _scan_for_tree()          # 扇形扫描 (azimuth offsets ±12°)
    │   ├── _recover_to_next_observation()  # 对准失败观察距离恢复
    │   └── ObservationOptimizer      # 碰撞IK/条件数(≤16.5)/关节余量(≥0.22rad)筛选
    │
    └── DownstreamActionMixin # 下游 Action：AlignTarget、Spray 的可靠调用
        ├── _run_downstream_action()  # 统一的 server 等待+超时+取消处理
        ├── _align_target()           # /vision/align_target 封装
        └── _spray_target()           # /spray/execute 封装
```

**关键设计决策**：
- `MotionControlState` 全局锁——stop/reset/resume 信号优先于所有任务推进
- `cancel_epoch` 机制——每次 cancel 递增版本号，旧轨迹自动失效
- 观察位姿不再使用固定关节角度，完全由 `ObservationOptimizer` 动态计算（距离/高度/方位角网格 + 实时 IK 筛选）

## 5. 控制链频率分层

```
C10 Camera (30 Hz)
    ↓ sensor_msgs/Image
YOLOv8n Tree Detect + Fruit Seg (30 Hz)
    ↓ Detection2DArray → Target2D
Visual Servo IBVS PID (30 Hz)
    ↓ geometry_msgs/TwistStamped
MoveIt Servo (100 Hz, publish_period=0.01)
    ↓ trajectory_msgs/JointTrajectory
ros2_control / arm_controller (100 Hz)
    ↓
Gazebo / 真实 Alicia-M 电机
```

## 6. 关键参数

### Mission Manager

| 参数 | 值 | 含义 |
|------|-----|------|
| `docking_lateral_offset` | 0.2 m | 道路停靠横向偏移 |
| 线速度停稳阈值 | 0.03 m/s | |
| 角速度停稳阈值 | 0.03 rad/s | |
| 稳定持续时间 | 1.0 s | 进入机械臂前门槛 |

### ObservationOptimizer

| 参数 | 值 |
|------|-----|
| 距离范围 | 0.90~1.50 m, 步长 0.10 |
| 相机高度范围 | 1.45~1.75 m, 步长 0.10 |
| 方位角偏移 | 0°, ±12° |
| 最大条件数 | 16.5 |
| 最小关节余量 | 0.22 rad |
| 优选关节余量 | 0.35 rad |
| 搜索超时 | 8.0 s |

### TargetRecenter

| 参数 | 值 |
|------|-----|
| 触发阈值 | 偏差 > 48 px |
| 最大旋转 | 20°/次 |
| 最大迭代 | 2 |
| 细化目标 | 24 px |
| 后重心稳定 | 0.20 s, 漂移 ≤ 4 px |

### IBVS (Visual Servo)

| 参数 | 仿真值 | 实机建议 |
|------|--------|---------|
| 控制频率 | 30 Hz | 30 Hz |
| `Kp XY` | 4.0 | 1.0~2.0 |
| `Kd XY` | 0.005 | 0.005 |
| 最终容差 | 1.5 px/轴 | 1.5 px/轴 |
| 稳定时间 | 0.5 s | 0.5 s |
| 对准超时 | 8.0 s | 8.0 s |
| 最大线速度 | 0.08 m/s | 0.04 m/s |
| 最大线加速度 | 0.60 m/s² | 0.30 m/s² |
| 命令模式 | `angular_xy` | `angular_xy` |

### MoveIt 轨迹

| 参数 | 仿真值 | 实机值 |
|------|--------|--------|
| velocity_scaling | 0.40 | **0.10** |
| acceleration_scaling | 0.50 | **0.10** |
| allowed_planning_time | 2.0 s | 2.0 s |
| collision_checking | false (仿真) | **true** (实机必须) |

## 7. 仿真 vs 实机差异

| 项目 | 仿真 (`system_sim.launch.py`) | 实机 (`real_system_mission.launch.py`) |
|------|------|------|
| `use_sim_time` | `true` | `false` |
| 底盘 | `ackermann_sim.py` (Twist→odom) | CAN bridge + wtb_car |
| LiDAR | pointcloud_to_laserscan | 真实 lslidar_driver |
| IMU | 无 | yesense_std_ros2（旧 fdilink_ahrs 仅保留回滚） |
| 相机 | Gazebo 插件 | usb_cam + C10 |
| ros2_control | gazebo_ros2_control/GazeboSystem | alicia_m_driver/AliciaHardwareInterface |
| base_frame | `alicia_base_link` | `base_link` |
| 速度缩放 | 0.40/0.50 | 0.10/0.10 |
| 碰撞检测 | false | true |
| MoveIt Servo publish_period | 0.01 (100Hz) | 0.01 (100Hz) |

## 8. 实机待办

| # | 任务 | 状态 |
|---|------|------|
| 1 | C10 相机内参标定（棋盘格） | 未开始 |
| 2 | 手眼标定 (tool0→camera_link) | 未开始 |
| 3 | 统一实机 launch 文件 | 代码已接入，待现场验收 |
| 4 | 实机配置子目录 (`config/real/`) | 已接入 |
| 5 | 底盘 CAN 通信验证 | 未开始 |
| 6 | 机械臂低速运动验证 | 未开始 |
| 7 | YOLO 真实图像推理验证 | 未开始 |
| 8 | 单树手动闭环 | 未开始 |
| 9 | 真实喷洒泵阀接入 | 未开始 |
| 10 | 联合调试与安全验收 | 未开始 |

## 9. 测试策略

1. **纯函数单元测试**：状态机、停靠计算、检测去重、目标关联、PID、限速与几何
2. **Fake ROS 闭环**：Nav2、Spray、Mission、Action 取消与恢复
3. **Launch 静态检查**：启动顺序、控制器依赖
4. **构建测试**：全部 24 包独立构建
5. **Gazebo 验收**：真实话题、TF、控制频率、误差与任务统计

```bash
cd ~/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --all  # 当前: 196 tests, 0 failures, 0 skipped
```

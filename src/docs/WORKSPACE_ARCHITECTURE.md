# WVCSC_S2Z_UTB_ARM 工作区架构

> 更新日期：2026-07-18
> 平台：ROS 2 Humble + Gazebo Classic 11 + Nav2 + MoveIt 2
> 主入口：`ros2 launch wvcsc_simulation system_sim.launch.py`

## 1. 系统目标

本工作区实现智能农林空地协同作业的核心闭环：

```text
无人机/Mock UAV 病树坐标
  → 小车生成停靠位姿并由 Nav2 导航
  → /odom 连续停稳确认
  → Alicia-M 观察果树
  → 两级 YOLO 检测树与健康/病果
  → 目标重心与 MoveIt Servo 图像平面 XY 对准
  → 逐个模拟喷洒
  → 返回观察位或 HOME
```

仿真与真机共用 ROS 接口。硬件驱动、真实相机和真实喷头通过稳定边界替换，不侵入任务状态机。

## 2. 25 个 ROS 包

### 2.1 上游与硬件基础包

| 包 | 职责 |
|---|---|
| `serial` | 串口基础库 |
| `can_bridge` | CAN 设备桥接 |
| `wtb_car_driver` | 阿克曼底盘驱动 |
| `fdilink_ahrs` | IMU/GNSS 驱动 |
| `lslidar_msgs`、`lslidar_driver` | 激光雷达消息与驱动 |
| `my_cartographer` | Cartographer 建图配置 |
| `my_navigation2` | 真机 Nav2 与 Qt 操作入口 |
| `pymoveit2` | Python MoveIt 2 公共客户端 |

### 2.2 Alicia-M 上游包

| 包 | 职责 |
|---|---|
| `alicia_m_descriptions` | Alicia-M URDF 与网格 |
| `alicia_m_driver` | 机械臂硬件驱动 |
| `alicia_m_moveit_config` | SRDF、运动学、控制器与规划配置 |
| `alicia_m_bringup` | 真机综合启动 |
| `alicia_m_calibration` | 标定工具 |
| `trajectory_retime_server` | 为 Alicia-M bringup 保留的轨迹重定时服务 |

### 2.3 WVCSC 项目包

| 包 | 职责 | 稳定边界 |
|---|---|---|
| `wvcsc_interfaces` | 项目消息、服务与 Action | 所有项目包共享 |
| `wvcsc_description` | 小车、机械臂、C10 的统一 Xacro | 仿真/真机复用 |
| `wvcsc_uav_gateway` | Mock/Replay 无人机任务源 | `/uav/disease_trees` |
| `wvcsc_mission_manager` | 导航、停稳、机械臂编排 | `/mission/*` |
| `wvcsc_arm_task` | 观察、重心、逐果作业、HOME | `/arm/execute_spray` |
| `wvcsc_rgb_vision` | 两级 YOLO、跟踪、目标锁定 | `/vision/*` |
| `wvcsc_visual_servo` | 30 Hz IBVS + 100 Hz Servo 图像平面 XY 控制 | `/vision/align_target` |
| `wvcsc_spray_controller` | 可取消的喷洒执行器边界 | `/spray/execute` |
| `wvcsc_c10_camera` | C10 真机采集、诊断、断线恢复 | 标准 Image/CameraInfo |
| `wvcsc_simulation` | Gazebo 果园、Nav2 仿真、统一 launch、数据采集 | `system_sim.launch.py` |

`wvcsc_web_ui` 已删除。任务操作保留 ROS 服务、Nav2 Qt 和各节点 `ros2 run` 入口。

## 3. 运行时分层

```text
任务输入层
  wvcsc_uav_gateway
        │ DiseaseTreeArray
        ▼
任务编排层
  wvcsc_mission_manager ── NavigateToPose ── Nav2
        │ ExecuteSpray
        ▼
机械臂任务层
  wvcsc_arm_task
    ├─ MoveIt 规划/执行
    ├─ wvcsc_rgb_vision
    ├─ wvcsc_visual_servo
    └─ wvcsc_spray_controller
        │
        ▼
设备与仿真层
  Gazebo / C10 / Alicia-M / 底盘驱动
```

职责约束：

- Mission Manager 不直接控制关节、相机或喷头。
- UAV Gateway 不生成导航位姿，只发布病树信息。
- RGB Vision 不发机械臂命令，只发布检测和目标。
- Visual Servo 不负责长轨迹规划，只输出受限微调命令。
- Spray Controller 独立保留，便于将仿真定时器替换为泵阀驱动。

## 4. 核心接口

### 4.1 任务与导航

| 名称 | 类型 | 用途 |
|---|---|---|
| `/uav/disease_trees` | `DiseaseTreeArray` | 病树任务列表 |
| `/mission/status` | `MissionStatus` | 任务状态与统计 |
| `/mission/plan` | `MissionPlan` | 停靠与任务计划 |
| `/mission/start` 等 | `std_srvs/Trigger` | start/pause/resume/cancel/reset |
| `/navigate_to_pose` | Nav2 Action | 小车导航 |
| `/odom` | `nav_msgs/Odometry` | 停稳确认 |

默认停靠横向偏移由 `DEFAULT_DOCKING_LATERAL_OFFSET=0.2 m` 统一定义，左右目标分别落在道路中心线两侧。

### 4.2 视觉与机械臂

| 名称 | 类型 | 用途 |
|---|---|---|
| `/arm/execute_spray` | `ExecuteSpray` Action | 一棵树的完整机械臂任务 |
| `/vision/tree_detections` | `Detection2DArray` | 树检测 |
| `/vision/fruit_detections` | `Detection2DArray` | 果实检测 |
| `/vision/target` | `Target2D` | 锁定目标与安全瞄准点 |
| `/vision/align_target` | `AlignTarget` Action | 图像平面 XY 对准 |
| `/spray/execute` | `Spray` Action | 喷洒执行 |

当前仿真喷洒保持目标时长并输出：

```text
喷洒动作进行中......duration=5.0s
喷洒动作成功执行
```

## 5. TF 与控制所有权

仿真 TF 主链：

```text
world → map → odom → base_footprint → base_link → alicia_base_link → ... → tool0 → camera
```

- 静态发布器负责 `world→map` 与仿真 `map→odom`。
- Ackermann 仿真节点独占 `odom→base_footprint`。
- `robot_state_publisher` 发布机器人固定/关节 TF。
- 仿真不启动 AMCL，避免第二个 `map→odom`。

MoveIt 负责观察、重心和 HOME 的碰撞规划。MoveIt Servo 只承担短距离图像对准；
Gazebo 采用 30 Hz IBVS Twist、100 Hz Servo JointTrajectory 和 100 Hz
`ros2_control`。当前关闭 Servo 在线碰撞缩放以避免同步碰撞计算阻塞控制链，
仍保留关节限位、奇异点保护和规划阶段碰撞检查。该参数不得直接用于真机。

## 6. 启动入口

主仿真：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 launch wvcsc_simulation system_sim.launch.py
```

无 GUI 验证：

```bash
ros2 launch wvcsc_simulation system_sim.launch.py gazebo_gui:=false use_rviz:=false
```

C10 真机：

```bash
ros2 launch wvcsc_c10_camera c10_camera.launch.py
```

Mock、Replay、视觉、任务、喷洒节点仍保留 `ros2 run` 入口。删除的独立 launch 不再作为支持入口。

## 7. 数据与模型

- 果实分割与树检测数据集共享同一批 C10 原始 PNG。
- seeds 50–53 为 train，seed 54 为 val。
- 数据采集、复制、验证逻辑位于 `wvcsc_simulation/yolo_seed_dataset.py`。
- YOLO 权重均安装在 `wvcsc_rgb_vision/share/.../models`，四个现有 `.pt` 文件必须保留原名和 SHA256。
- Ultralytics 运行环境与 ROS 主环境隔离，通过 launch 中的 Python 可执行文件参数接入。

## 8. 包收敛边界

以下边界有意保持独立：

- RGB Vision：隔离 YOLO/PyTorch 依赖。
- Visual Servo：隔离 30 Hz 图像控制与 100 Hz Servo 输出链。
- Arm Task：隔离长时动作与恢复状态机。
- Spray Controller：保留真喷头替换点。
- UAV Gateway：保留 Live UAV 替换点。
- C10 Camera：隔离设备生命周期和诊断。

`trajectory_retime_server` 由未修改的 `alicia_m_bringup` 和当前 WVCSC 仿真
的 Alicia 轨迹适配链共同使用，因此保留在依赖图中。受保护的上游/硬件包不参与本轮重构。

## 9. 当前验收边界

代码侧要求：

- 全工作区 Python 测试无重名、无失败。
- 相关包可独立 `colcon build/test`。
- `system_sim.launch.py --show-args` 和 C10 launch 可解析。
- 四个权重源目录与安装目录哈希一致。

运行时仍需用户在 Gazebo 验收：

- 导航、停稳、观察、目标重心、视觉伺服、5 秒喷洒和 HOME。
- 图像对准双轴误差不超过 4 px，保持至少 0.5 s。
- 对准中位时间不超过 5 s，最大不超过 8 s。
- 连续三轮无 unresolved、alignment failure 或 skipped target。

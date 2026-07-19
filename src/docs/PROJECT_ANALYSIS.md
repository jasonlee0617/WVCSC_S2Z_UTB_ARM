# WVCSC_S2Z_UTB_ARM 项目分析

> 更新日期：2026-07-19
> 详细架构：[WORKSPACE_ARCHITECTURE.md](WORKSPACE_ARCHITECTURE.md)
> 仿真执行手册：[WVCSC_S2Z_UTB_ARM_Codex_仿真任务闭环实施方案.md](WVCSC_S2Z_UTB_ARM_Codex_仿真任务闭环实施方案.md)

## 1. 项目定位

本项目面向智能农林病虫害空地协同防治：无人机提供疑似病树坐标，阿克曼小车自主停靠，Alicia-M 机械臂利用 C10 RGB 相机识别病果并进行图像视觉伺服，最后逐个执行喷洒。

当前交付重点是 Gazebo 可复现闭环，不包括真实飞控、泵阀、液位和真实喷幅控制。

## 2. 当前闭环

```text
Mock/Replay UAV
  → DiseaseTreeArray
  → Mission Manager 生成 0.2 m 横向偏移停靠点
  → Nav2 NavigateToPose
  → /odom 连续停稳
  → Arm ExecuteSpray
  → 动态观察位姿与扇形扫描
  → tree YOLO detect + fruit YOLO segment
  → 病果去重、逻辑目标锁定、目标重心
  → 30 Hz IBVS Twist → 100 Hz MoveIt Servo 图像平面 XY 对准
  → Spray Action 保持 5 s
  → 返回观察位继续下一病果
  → HOME
```

### 已建立能力

| 子系统 | 状态 | 说明 |
|---|---|---|
| 复合机器人描述 | 已完成 | 小车、Alicia-M、tool0、C10 与 ros2_control 统一 Xacro |
| 果园场景 | 已完成 | seed 固定资产，5 果/树、2–3 病果、稀疏叶片 |
| 小车仿真 | 已完成代码链 | Ackermann 速度约束、odom 与 TF 所有权明确 |
| 任务管理 | 已完成 | 顺序执行、停稳、fail-safe、skip、cancel/reset |
| 机械臂任务 | 已完成代码链 | 动态观察、目标重心、逐果作业、HOME |
| 两级 YOLO | 已接入 | 树检测、果实分割、检测去重、目标锁定与模板短时跟踪 |
| 视觉伺服 | 已接通 | 30 Hz Twist、100 Hz JointTrajectory、2 px 稳定门槛、状态保护 |
| 喷洒执行器 | 已完成仿真边界 | 定时 5 s，无夹爪开闭副作用 |
| C10 真机入口 | 第一版完成 | by-id、标准图像话题、诊断与 respawn |

### 尚未通过的最终验收

最新运行基线仍未证明“所有病果快速、高精度对准并完成喷洒”。代码已经修复 Servo 吞吐、检测重复、目标生命周期、低置信度准入和停止服务问题，但需要新的 Gazebo bag 验证：

- 双轴最终误差均不超过 4 px；
- 连续稳定至少 0.5 s；
- 对准中位时间不超过 5 s、最大不超过 8 s；
- 所有唯一病果均完成喷洒；
- 连续三轮 `unresolved=0`、`alignment_failures=0`、`skipped_targets=0`。

因此当前状态应描述为“代码闭环已接通，真实性能验收未完成”，不能仅根据 `MISSION_COMPLETED` 判定成功。

## 3. 技术架构评估

### 3.1 合理之处

1. **任务源与执行器解耦**
   UAV 只发布病树语义信息，停靠位姿由 Mission Manager 生成，避免外部数据源直接控制底盘。

2. **长轨迹与微调分层**
   MoveIt 负责观察、重心和 HOME，MoveIt Servo 负责局部 XY 对准，符合机械臂作业控制层次。

3. **喷洒接口独立**
   `/spray/execute` 可从仿真定时器替换为泵阀驱动，不需要重写视觉或机械臂状态机。

4. **视觉环境隔离**
   YOLO/PyTorch 运行时与系统 ROS Python 隔离，降低 NumPy、OpenCV 和 Torch ABI 污染风险。

5. **失败安全**
   目标丢失、重关联歧义、IK 不可达、Servo 安全状态、取消和 HOME 失败均不会继续喷洒。

### 3.2 当前风险

| 风险 | 影响 | 当前措施 | 后续动作 |
|---|---|---|---|
| YOLO 中心视角泛化不足 | 重心后目标丢失 | 置信度门控、模板短时跟踪 | 增补中心视角并按 seed 隔离重训 |
| 单目只有像素误差 | 无法直接证明喷嘴空间误差 | 固定作业距离、4 px 门槛 | 标定相机—喷嘴外参与喷距 |
| Gazebo 关闭 Servo 在线碰撞缩放 | 仿真安全模型弱于真机 | 保留规划碰撞、限位和奇异保护 | 真机重新启用并测试碰撞计算预算 |
| 底盘/机械臂硬件依赖不完整 | 本机全量构建受阻 | 项目包可独立构建测试 | 补齐合法厂商 `libcontrolcan.so` |
| 真实喷头未接入 | 无法验证喷幅与药量 | 独立 Spray Action 边界 | 增加泵阀、流量、液位、急停反馈 |

## 4. 包收敛结果

工作区从 26 个 ROS 包收敛为 25 个：

- 当前不保留独立 Web UI；任务操作统一到 ROS 服务和受保护的 Nav2 Qt 前端。
- `trajectory_retime_server` 仍由未修改的 `alicia_m_bringup` 以及 WVCSC 仿真
  的 Alicia 轨迹适配链使用，因此不从当前依赖图移除。
- `wvcsc_arm_task` 和 `wvcsc_simulation` 当前仍通过 Alicia 轨迹适配链使用重定时服务。
- 删除六个项目独立 launch 和 retime 独立 launch，支持入口集中到系统 launch、C10 launch 和 `ros2 run`。
- Arm Adapter 的 Cartesian retime 校验和 open/close gripper 能力继续保留，避免破坏
  当前仿真、复位和测试接口。
- 将视觉 PID 收敛为标准库实现的二维控制器，移除 NumPy 运行依赖。
- 合并 fruit/tree 数据集的 split 验证重复逻辑。

保留独立包：

- `wvcsc_rgb_vision`：YOLO 依赖隔离；
- `wvcsc_visual_servo`：实时控制隔离；
- `wvcsc_arm_task`：长时任务与恢复；
- `wvcsc_spray_controller`：真喷头替换边界；
- `wvcsc_uav_gateway`：Live UAV 替换边界；
- `wvcsc_c10_camera`：设备生命周期与诊断。

## 5. 关键参数

### Mission Manager

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `docking_lateral_offset` | 0.2 m | 靠近目标侧的道路停靠偏移 |
| 线速度停稳阈值 | 0.03 m/s | 连续停稳检测 |
| 角速度停稳阈值 | 0.03 rad/s | 连续停稳检测 |
| 稳定持续时间 | 1.0 s | 进入机械臂阶段前门槛 |

### Visual Servo（仅 Gazebo）

| 参数 | 当前值 |
|---|---:|
| IBVS / MoveIt Servo / ros2_control | 30 / 100 / 100 Hz |
| `Kp XY` | 2.5 |
| `Kd XY` | 0.005 |
| 最大线速度 | 0.08 m/s |
| 最大线加速度 | 0.60 m/s² |
| 最终容差 | 2 px/轴 |
| 稳定持续时间 | 0.5 s |
| 对齐超时 | 8 s |

这些参数不得直接用于真机。真机应恢复在线碰撞检查并从更低速度重新整定。

## 6. 测试策略

代码测试分层：

1. 纯函数：状态机、停靠计算、检测去重、目标关联、PID、限速与几何。
2. Fake ROS 闭环：Nav2、Spray、Mission、Action 取消与恢复。
3. Launch 静态检查：启动顺序、Gazebo 重力切换和控制器依赖。
4. 构建测试：相关项目包与保留的 retime 核心包。
5. Gazebo 验收：真实话题、TF、控制频率、误差与任务统计。

手动 rosbag 使用 `wvcsc_visual_servo/scripts/record_servo_bag.sh`。不维护额外离线报告工具；用户提供 bag 后按原始消息和终端日志分析。

## 7. 推荐实施顺序

1. 完成本轮删减后的全包构建与测试。
2. 在 Gazebo 运行单棵树并手动录 bag，确认 Twist 为 `27–33 Hz`、
   JointTrajectory 为 `90–110 Hz`，且 Gazebo 实时率不低于 `0.95`。
3. 先验收目标锁定和重心，再验收 PID 收敛，最后验收逐果喷洒。
4. 若低置信度仍阻断，优先重训模型，不继续提高 PID。
5. 完成 C10 内参、相机—tool0—喷嘴外参和固定喷距标定。
6. 接入真实泵阀、液位与急停反馈。
7. 最后进入真实底盘与机械臂联合调试。

## 8. 完成定义

项目不能以“节点均启动”或“任务列表处理完”作为完成。至少满足：

- TF 无重复父节点；
- 导航失败不触发机械臂；
- 视觉失败不触发喷洒；
- 每颗病果只进入一次喷洒队列；
- cancel 后无残留 Goal；
- 喷洒后返回安全位；
- 连续三轮完整闭环达到既定速度、精度和任务统计门槛。

# WVCSC 小车 + Alicia-M + C10 视觉喷洒仿真闭环实施方案

> 更新日期：2026-07-15  
> 当前结论：代码、构建、静态检查和 71 项单元测试已通过；Gazebo 相机图像、CameraInfo、病斑检测和安全停机已实测。最后一项仍需手动验收：使用新观察姿态完成四目标视觉伺服闭环。

## 1. 当前目标

在同一个 Gazebo 世界和同一个复合机器人模型中完成：

```text
Mock 无人机回传四棵病树
  → Nav2 自动导航并停稳
  → Alicia-M 到达左/右观察姿态
  → 末端 C10 发布 RGB 图像和 CameraInfo
  → 临时颜色分割输出病斑位置和置信度
  → MoveIt Servo 只修正图像平面 X/Y，保持喷距
  → 模拟喷洒
  → 返回全零 HOME
  → 处理下一目标
```

固定停车喷距范围暂定 `0.8–1.5 m`。仿真使用 `spray_standoff_distance=2.4 m`，末端到病斑的几何距离约 `1.2 m`。

## 2. 已实现内容

| 模块 | 当前实现 |
|---|---|
| C10 模型 | 参考 Moveit2YoloObb 的相机网格，增加简化碰撞体、可调安装外参和许可证 |
| 复合 URDF | `tool0 → camera_link → camera_color_optical_frame`，同时增加 `spray_nozzle_link` |
| Gazebo 相机 | 1280×720、30 Hz、HFOV 1.8、RGB、零畸变、Gaussian noise 0.007 |
| 实机相机包 | `wvcsc_c10_camera` 封装 `usb_cam`，使用 `/dev/v4l/by-id`、进程重启和诊断 |
| 仿真病斑 | 四棵目标树布置可见病斑模型，其他树保持原场景 |
| 视觉检测 | 临时 HSV 分割发布 `Detection2DArray` 和选中 `Target2D`，后续替换 YOLO Seg |
| 视觉伺服 | `wvcsc_visual_servo` 通过 MoveIt Servo 发布末端速度，只控制光学坐标 X/Y，Z 恒为 0 |
| 任务闭环 | 无目标/超时：HOME 后跳过当前目标；奇异、碰撞、Servo 停止失败：锁定并终止任务 |

主要接口：

| 接口 | 类型 | 说明 |
|---|---|---|
| `/camera/camera/color/image_rect_raw` | `sensor_msgs/Image` | 仿真和实机统一 RGB 图像 |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 仿真和实机统一相机内参 |
| `/vision/pest_detections` | `vision_msgs/Detection2DArray` | 所有病斑候选 |
| `/vision/target` | `wvcsc_interfaces/Target2D` | 当前任务选中的视觉目标 |
| `/vision/align_target` | `wvcsc_interfaces/AlignTarget` | 视觉对准 Action |
| `/servo_node/status` | `std_msgs/Int8` | MoveIt Servo 安全状态 |
| `/arm/execute_spray` | `wvcsc_interfaces/ExecuteSpray` | 观察、对准、喷洒、HOME |
| `/mission/status` | `wvcsc_interfaces/MissionStatus` | 任务状态与目标计数 |

相机仿真/参考内参：

```text
width=1280, height=720, fps=30
fx=fy=507.872735
cx=640.5, cy=360.5
D=[0,0,0,0,0]
frame_id=camera_color_optical_frame
```

这些参数来自当前参考模型，不是 C10 实物标定结果。实机安装后必须重新做内参和手眼外参标定。

## 3. 当前验证状态

已完成：

- `colcon build --packages-up-to wvcsc_simulation`：16 个相关包构建通过；
- Gazebo C10 相机已实测约 30 Hz，图像为 1280×720，CameraInfo 正常；
- 相机话题改为 `BEST_EFFORT + KEEP_LAST(1)` 后，大图像能够持续送达检测节点；
- 病斑模型可见，临时检测置信度可达到 `0.99`；
- 检测、任务 ID、树 ID、视觉 Action 和故障语义已接通；
- 单元测试共 71 项通过；
- Xacro 展开和 `check_urdf` 通过，TF 根为 `base_footprint`。

发现并修复的最后一个运行问题：旧观察姿态进入 MoveIt Servo 后接近奇异点，Servo 返回硬停止并正确锁定任务。现已替换为离线求解的新姿态：

```yaml
observe_left_pose:  [ 1.886845, -1.463996, -1.033531,  0.597978, 1.272105, -2.261712]
observe_right_pose: [-1.882066, -1.471510, -1.031065, -0.585215, 1.288457, -0.891742]
```

离线数值雅可比条件数约为 `12.33` 和 `12.20`，低于当前减速阈值 `17.0`。它们还需要按第 6 节进行 Gazebo 实际规划、碰撞和闭环验收，不能仅凭离线指标判定完成。

## 4. 关键安全规则

- 不使用单目 RGB 虚构深度；喷距由停车位置和观察姿态保证。
- 视觉伺服只修正图像 X/Y，光轴 Z 速度始终为 0。
- `fine_tolerance=8 px`，连续 `10` 帧满足才算对准成功。
- 检测置信度低于 `0.70`、目标超过 `0.2 s` 未更新时不发送运动命令。
- MoveIt Servo 状态 `2/4/5` 或未知非零状态按危险处理：发零速、发布 `stop`、锁定机械臂、任务失败。
- 不降低 `lower_singularity_threshold=17` 或 `hard_stop_singularity_threshold=30` 来掩盖姿态问题。
- 视觉目标缺失或普通超时：停止 Servo、回 HOME、标记当前树跳过，然后继续下一棵。
- HOME、控制器、Servo 停止或安全恢复失败：保持锁定，不继续下一目标。

## 5. 仿真与实机的边界

仿真和实机共用：

- 复合 URDF 的机械结构、C10/喷嘴坐标系和可调安装外参；
- 相同图像、CameraInfo、检测、视觉目标、视觉 Action 和任务接口；
- 相同 MoveIt 规划组 `arm`、关节 `joint1–joint6` 和安全状态机。

仅仿真使用：

- Gazebo 相机插件、病斑颜色分割、病斑模型和 Ackermann 仿真；
- 参考内参和临时安装外参。

实机前必须完成：

- `ls -l /dev/v4l/by-id/` 确认 C10 的真实持久设备路径；
- 1280×720 实机内参标定和畸变参数更新；
- 相机支架固定后的手眼外参标定；
- C10 MJPG/YUY2、曝光、时间戳、断线恢复压力测试；
- Alicia-M 厂家关节速度/加速度限制确认；
- 用真实 YOLO Seg 权重替换仿真颜色分割。

## 6. 最终手动验收步骤

### 6.1 构建和加载环境

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to wvcsc_simulation
source install/setup.bash
```

确认没有旧进程。若有输出，应回到对应终端按 `Ctrl+C` 后再启动：

```bash
pgrep -af '[r]os2 launch wvcsc_simulation|[g]zserver|[g]zclient'
```

### 6.2 启动完整四目标闭环（终端 1）

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wvcsc_simulation system_sim.launch.py \
  use_nav2:=true use_rviz:=false \
  use_mock_uav:=true use_replay_uav:=false \
  use_mission_manager:=true auto_start_mission:=true \
  use_web_ui:=false use_mock_vision:=false use_color_vision:=true \
  use_vision_alignment:=true use_spray_simulator:=false \
  use_spray_action:=false spray_standoff_distance:=2.4
```

### 6.3 相机与视觉监控（终端 2）

每条持续输出命令使用独立终端：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 topic hz /camera/camera/color/image_rect_raw \
  --qos-reliability best_effort
```

```bash
ros2 topic echo /camera/camera/color/camera_info --once \
  --qos-reliability best_effort
```

```bash
ros2 topic echo /vision/pest_detections
```

```bash
ros2 topic echo /vision/target
```

### 6.4 Servo 与任务监控（终端 3）

```bash
source /opt/ros/humble/setup.bash
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 action info /vision/align_target
ros2 topic echo /servo_node/status
```

另开终端：

```bash
ros2 topic echo --no-daemon \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /mission/status
```

需要检查末端坐标链时运行：

```bash
ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame
ros2 run tf2_ros tf2_echo alicia_base_link spray_nozzle_link
```

### 6.5 必须观察到的顺序

四个目标依次为：

```text
tree_01 left  → tree_02 right → tree_03 left → tree_04 right
```

每个目标必须出现：

```text
NAVIGATING
→ VERIFYING_STOP
→ ARM_SPRAYING / MOVING_TO_OBSERVE
→ ALIGNING
→ SPRAYING
→ RETURNING_HOME
→ TARGET_COMPLETED
```

最终必须是：

```text
MISSION_COMPLETED
completed_targets=4
skipped_targets=0
nav_goal_active=false
arm_goal_active=false
```

### 6.6 验收标准

- [ ] 相机稳定为 1280×720、约 30 Hz；
- [ ] CameraInfo 的 `frame_id` 和内参符合第 2 节；
- [ ] 四棵目标均有有效检测，置信度不低于 `0.70`；
- [ ] `/servo_node/status` 不出现硬停止状态 `2`；
- [ ] 左右观察姿态均能通过 MoveIt 规划和碰撞检查；
- [ ] 每个目标最终像素误差均不超过 `8 px`，并连续稳定 10 帧；
- [ ] 视觉伺服过程中不改变光轴 Z 喷距；
- [ ] 四次喷洒后机械臂均回到全零 HOME；
- [ ] 最终 `MISSION_COMPLETED`，完成数为 4、跳过数为 0；
- [ ] 退出后无遗留 Nav2、Arm、Servo 或 Spray Goal。

如果再次出现：

```text
Very close to a singularity, emergency stop
```

不要放宽 Servo 阈值。保存终端输出、当前目标、关节角和 `/servo_node/status`，再调整对应观察姿态。

## 7. 实机 C10 启动方式（硬件到位后）

先安装系统依赖并确认设备路径，然后启动：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/WVCSC_S2Z_UTB_ARM/install/setup.bash

ros2 launch wvcsc_c10_camera c10_camera.launch.py \
  video_device:=/dev/v4l/by-id/实际设备名
```

检查：

```bash
ros2 topic hz /camera/camera/color/image_rect_raw \
  --qos-reliability best_effort
ros2 topic echo /diagnostics
```

真实相机与 Gazebo 相机不能同时发布到同一组话题。

## 8. 参考资源与许可证

- C10 相机网格参考 `laoxue888/Moveit2YoloObb`，原许可证保存在 `wvcsc_description/licenses/Moveit2YoloObb-LICENSE`；
- 果树模型为非商业科研/比赛演示资源，归属与许可证保存在 `wvcsc_simulation/models/apple_tree/`；
- 视觉伺服只迁移 Fairino 工作区中通用 PID、限幅、状态策略、目标预测和参数结构，项目控制逻辑位于 `wvcsc_visual_servo`；
- 当前颜色分割是仿真占位实现，不作为最终病虫害模型。

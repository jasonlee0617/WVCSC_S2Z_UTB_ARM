# WVCSC 实机 Bringup

先加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

## 完整任务：Qt 选点、导航、喷洒

```bash
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

完整任务只接受 Qt 创建的任务：不再支持 YAML/命令行采点和 `mission_mode`。在 RViz 设置
`2D Pose Estimate` 后，在 Qt 点击“确认当前起点”。通行点或终点用一个 `2D Goal` 记录；
病株点先点击停车位，再点击树中心。Qt 可保存/加载 JSON 任务。

初始定位不准或误记点位时，点击“重新定位并清空任务”。它会清空起点和任务列表、请求 AMCL
全局重定位；随后重新在 RViz 点击 `2D Pose Estimate`，再确认新起点。

完整 Qt 的下方右侧是原始 `sensor_msgs/Image` 图像查看器，会动态列出相机和 YOLO 图像话题，
优先显示：

- `/camera/color/image_raw`
- `/vision/diseased_target_debug_image`

视觉伺服超时、目标过期或输出停滞可按实机配置执行受控回退喷洒；伺服硬安全停止、碰撞、
关节限位及奇异性门控不能绕过。

## 真实底盘与继电器联调（Qt + 假机械臂）

```bash
ros2 launch wvcsc_bringup real_vehicle_relay_qt_test.launch.py
```

该入口启动真实底盘、LiDAR、IMU、EKF、Nav2 和真实继电器。Qt 负责选点与提交任务；
`fake_arm_spray_action.py` 代替真实机械臂，并实际脉冲第 2 路继电器。它不启动 MoveIt、C10、
YOLO 或视觉伺服。

## 单机械臂喷洒测试

启动后端服务器（不打开 Qt）：

```bash
cd ~/WVCSC_S2Z_UTB_ARM/src
./run_real_arm_spray_server.sh
```

在另一个已加载工作区环境的终端启动 Qt：

```bash
ros2 run wvcsc_bringup arm_spray_test_qt.py
```

也可以用原始 launch 一次启动后端和 Qt：

```bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

单臂 Qt 只执行一个目标。两种模式都提供病株侧位、喷洒时长和手动工作距离；IK 模式额外
显示机械臂基座到病株的平面距离，`joint_preset` 模式不需要该距离。工作距离只用于喷嘴
瞄准平面标定，不替代机械臂碰撞、限位和奇异性检查，也不会写入完整导航任务。界面还提供
复位、HOME 完成后解锁、Action 阶段、进度、结果和可折叠的相机/YOLO 图像。

## 建图与单独导航

```bash
ros2 launch wvcsc_bringup real_cartographer.launch.py
ros2 launch wvcsc_bringup real_navigation.launch.py
```

实机默认 C10 为 `/dev/video0`，Alicia-M 串口为 `/dev/ttyACM0`；其他设备通过 launch 参数
显式覆盖。完整任务启动前会检查地图、标定、YOLO 环境、相机、机械臂和继电器配置。

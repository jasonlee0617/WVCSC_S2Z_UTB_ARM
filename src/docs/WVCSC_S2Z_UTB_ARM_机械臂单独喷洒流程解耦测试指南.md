# WVCSC 单机械臂 Qt 喷洒测试

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

此入口启动 Alicia-M、C10、YOLO、MoveIt、Visual Servo、喷洒执行器和第 2 路继电器；不启动
底盘、LiDAR、IMU、EKF、AMCL、Nav2 或 MissionManager。车辆必须保持停稳。

Qt 是唯一的测试入口：选择观察模式、病株侧位、喷洒时长和工作距离，点击“启动”。IK 模式额外
填写基座到病株距离；`joint_preset` 模式仅使用侧位和固定观测姿态。界面显示 Action 阶段、进度、
结果，并在右侧动态选择原始相机或 YOLO 图像话题。

出现异常或需要结束本次测试时点击“复位”。界面先停止并取消运动，再请求 HOME；HOME 物理动作
成功后 `/motion_control/state` 自动回到 `RUNNING`，无需人工解锁即可启动下一次测试。HOME 失败时
仍保持不可执行状态。现场物理急停始终优先。

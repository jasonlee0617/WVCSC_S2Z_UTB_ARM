# WVCSC 单机械臂 Qt 喷洒测试

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

此入口启动 Alicia-M、C10、YOLO、MoveIt、Visual Servo、喷洒执行器和第 2 路继电器；不启动
底盘、LiDAR、IMU、EKF、AMCL、Nav2 或 MissionManager。车辆必须保持停稳。

Qt 是唯一的测试入口：填写树 ID、相对 `alicia_base_link` 的 X/Y/Z 和喷洒时长，点击
“执行单目标喷洒”。`+X` 为车头前方、`+Y` 为车体左侧。界面显示 Action 阶段、进度、结果，
并在右侧动态选择原始相机或 YOLO 图像话题。

取消当前任务使用“取消 Action”。出现异常时点击“停止并回 HOME”；界面先停止并取消运动，再
请求 HOME。仅当 `/motion_control/state` 显示 `HOME_LOCKED` 后，“HOME 完成后解锁”才可用。
现场物理急停始终优先。

# WVCSC 实车 Qt 任务与继电器验收

完整任务入口：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"
```

任务点只在 Qt 中创建：RViz 完成 `2D Pose Estimate` 后确认起点；一次 `2D Goal` 记录通行/终点，
病株点先记录停车位、再点击树中心。误记或重新定位时点击“重新定位并清空任务”，重新完成
RViz 初始定位后再记录起点。Qt JSON 可保存和加载。

完整任务界面右侧显示动态发现的原始 `sensor_msgs/Image` 话题，优先包括 C10 图像、树检测图和
病株调试图。视觉伺服超时会在当前位置强制喷洒、回 HOME 后继续后续点；碰撞、限位与无法确认
停止仍会触发安全门控。

真实底盘/继电器联调入口：

```bash
ros2 launch wvcsc_bringup real_vehicle_relay_qt_test.launch.py
```

该入口使用真实车辆、导航和继电器，Qt 选点和 mission_manager 执行任务；假机械臂 Action
实际脉冲第 2 路继电器。它不启动真实机械臂、相机、YOLO 或视觉伺服。

# C10 眼在手 (Eye-in-Hand) 手眼标定指南

> 适用环境：ROS 2 Humble + Gazebo Classic 11 / 真实 Alicia-M + C10 相机
> 标定方法：easy_handeye2 + ArUco marker (DICT_5X5_250 id=1, 90×90 mm)

---

## 1. 仿真自动化标定

### 1.1 一键启动

```bash
cd /home/robot/WVCSC_S2Z_UTB_ARM
source install/setup.bash
ros2 launch wvcsc_simulation calibration_sim.launch.py
```

### 1.2 启动方式

```bash
# 终端 1: 仿真环境 (Gazebo + Robot + ArUco + handeye_server)
ros2 launch wvcsc_simulation calibration_sim.launch.py

# 终端 2: 标定采集器 (与实机操作完全相同)
ros2 run wvcsc_calibration auto_calibration_collector
# 按 s 开始，按 q 取消
```

### 1.3 自动化流程

```
终端 1: Gazebo + Robot + ArUco + handeye_server 启动 (一次性)
终端 2: 按 s → auto_calibration_collector:
      1. 生成 17 个观察姿态 (覆盖 ±30° cone)
      2. 每个姿态: MoveIt 移动 → ArUco 检测 → TakeSample
      3. ComputeCalibration → SaveCalibration
      4. 输出标定结果到 ~/.ros/easy_handeye2/
```

### 1.4 标定码位置

- ArUco marker: `model://aruco_marker`
- 世界坐标: `(0.20, 0, 0.62)` — arm_mount_link 正前方 20cm
- 尺寸: 90×90 mm, DICT_5X5_250 id=1

### 1.5 输出文件

```
~/.ros/easy_handeye2/
  handeye_calibration.yaml   ← 手眼变换矩阵 (tool0 → camera_color_optical_frame)
  samples.yaml                ← 17 个采样点的原始数据
```

---

## 2. 实机标定

### 2.1 准备工作

1. **打印 ArUco 标定码**
   - 字典: DICT_5X5_250, id=1
   - 物理尺寸: 90×90 mm（贴于 100×100 mm 硬板上）
   - 纸张: 哑光/无光面，避免反光

2. **固定标定码**
   - 位置: arm_mount_link 正前方 20cm (约机器人 base_link 前方 20cm)
   - 高度: 与 C10 相机安装高度齐平 (~62cm 距地面)
   - 固定: 使用三脚架或刚性支架，保证采样期间不移动

3. **检查硬件连接**
   ```bash
   # C10 相机
   ls /dev/v4l/by-id/usb-Synria_C10-video-index0
   # Alicia-M 机械臂串口
   ls /dev/ttyACM0
   ```

### 2.2 启动标定环境

```bash
# 终端 1: 传感器 + 机械臂
ros2 launch wvcsc_bringup real_sensors.launch.py
ros2 launch wvcsc_bringup real_arm.launch.py

# 终端 2: ArUco 检测 + easy_handeye2
ros2 launch wvcsc_calibration c10_handeye.launch.py

# 终端 3: 采集器
ros2 run wvcsc_calibration auto_calibration_collector
```

### 2.3 手动采样（备选）

如果自动采样失败，可在 `auto_calibration_collector` 终端中按 `s` 手动触发单次采样：

```
  s / Enter  → 触发一次采样
  q          → 取消当前 session
  Ctrl-C     → 退出
```

每个采样点要求：
- ArUco marker 在相机视野中且检测置信度 >0.8
- 机械臂处于静止状态（joint_states 5 帧内变化 <0.01 rad）
- 采集 17 个不同姿态（覆盖不同距离、角度）

### 2.4 检查标定质量

```bash
ros2 run wvcsc_calibration calibration_quality \
  ~/.ros/easy_handeye2/handeye_calibration.yaml
```

验收标准：
- 平移残差 <5 mm
- 旋转残差 <0.5°
- 条件数 <100

---

## 3. 标定结果接入

### 3.1 更新 bringup 配置

将标定结果写入 `wvcsc_bringup/config/real/c10_calibration.yaml`：

```yaml
# c10_calibration.yaml — 手眼标定结果
# tool0 → camera_color_optical_frame
translation:
  x: -0.055  # 来自标定
  y: 0.0
  z: -0.100
rotation:     # xyzw quaternion
  x: 0.0
  y: -0.707
  z: 0.0
  w: 0.707
```

### 3.2 验证标定接入

```bash
ros2 launch wvcsc_bringup system_real.launch.py
# 检查 TF 树: tool0 → camera_color_optical_frame
ros2 run tf2_tools view_frames
```

---

## 4. 故障排查

| 症状 | 原因 | 解决 |
|------|------|------|
| ArUco 检测不到 | 标定码反光/遮挡/尺寸不匹配 | 用哑光纸、确认 DICT_5X5_250 id=1、确认 marker_size=0.07 |
| 采样 0/17 | auto_collector 没收到 ArUco 话题 | 检查 `/aruco_markers` 是否发布、topic 名称是否一致 |
| ComputeCalibration 失败 | 样本姿态不够多样 | 增加采样数到 25、确保覆盖 ±30° 角度范围 |
| 真机标定残差大 | 标定码移动了 / 相机内参不准 | 重新标定 C10 内参 (camera_calibration) |
| TF 不连续 | tool0 → camera_link 用了临时值 | 确认 URDF 中 c10_mount_xyz 与标定一致 |

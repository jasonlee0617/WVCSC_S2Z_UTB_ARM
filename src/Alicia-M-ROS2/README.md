# Alicia-M-ROS2

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

## 项目简介 | Project & Feature Overview

`Alicia-M-ROS2` 是用于控制 Alicia-M 系列机械臂的 ROS2 工作区。项目基于 ROS 2 Humble，提供从底层驱动到上层规划的完整控制链路，覆盖硬件启动、MoveIt 运动规划、模型可视化与手眼标定等核心能力。

主要功能：

- `ros2_control` 硬件驱动集成（串口通信）
- MoveIt2 规划与执行支持
- 机械臂模型显示与 RViz 可视化
- 手眼标定（眼在手上，适配intel realsense d405）与标定结果验证
- Python 脚本示例（如 Pick and Place）
- (待开发)物块分拣工作流（2D抓取）
- (待开发)基于 GraspGen（NVIDIA 2025）的 6D 抓取工作流。

## 项目结构

```text
src/
├── alicia_m_bringup/          # 机械臂硬件与 MoveIt 启动入口
├── alicia_m_calibration/      # 手眼标定与验证脚本
├── alicia_m_descriptions/     # URDF、mesh、RViz 显示配置
├── alicia_m_driver/           # ros2_control 硬件驱动（串口通信）
├── alicia_m_grasp_6d/         # 6D 抓取模块（待开发）
├── alicia_m_moveit_config/    # MoveIt2 配置与控制器配置
└── examples/                  # MoveIt 示例脚本
```

## 安装方式

### 1. 环境要求

- Ubuntu 22.04
- ROS2 Humble

### 2. 依赖安装

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 编译工作区

在工作区根目录（包含 `src/`）执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 基本使用（启动机械臂）

设置临时串口权限：

```bash
sudo chmod 666 /dev/ttyACM*
```

加载工作区环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```



### Option 1. 启动硬件 + MoveIt（推荐）

```bash
ros2 launch alicia_m_bringup moveit_hardware.launch.py
```

### Option 2. 单独启动硬件控制

```bash
ros2 launch alicia_m_bringup hardware.launch.py
```

### Option 3. 启动仿真 + MoveIt

```bash
ros2 launch alicia_m_bringup moveit_sim.launch.py
```
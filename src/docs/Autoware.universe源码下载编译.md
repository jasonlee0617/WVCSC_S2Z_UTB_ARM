# Autoware.universe 源码下载编译

本文档以 WVCSC 工控机为主场景，目标是让 `/home/eisa/autoware` 成为稳定可用的 Autoware 工作区。

## 1. 基线环境

- Ubuntu 22.04
- ROS 2 Humble
- Autoware 工作区：`/home/eisa/autoware`
- WVCSC 工作区：`/home/eisa/WVCSC_S2Z_UTB_ARM`
- 模型与数据目录：`/home/eisa/autoware_data`
- `acados` 安装目录：`/home/eisa/.local/acados`

## 2. 工控机编译原则

工控机和台式开发机不一样，建议直接接受下面这些约束：

- 不要开太大的并行度
- 优先用纯终端 / `tmux` / SSH 编译
- 建议顺序执行 `colcon`
- 建议设置 `MAKEFLAGS=-j2`

推荐原因：

- Autoware 编译体量大
- `TensorRT` / `PCL` / `CUDA` 相关目标在高并行时更容易卡住桌面
- 工控机一旦内存吃满，体验会很差

## 3. 准备目录

```bash
mkdir -p /home/eisa/autoware_data
mkdir -p /home/eisa/autoware_data/ml_models
```

## 4. 准备环境

```bash
source /opt/ros/humble/setup.bash

export CMAKE_PREFIX_PATH="/home/eisa/.local/acados:${CMAKE_PREFIX_PATH}"
export ACADOS_SOURCE_DIR="/home/eisa/.local/acados"
export LD_LIBRARY_PATH="/home/eisa/.local/acados/lib:${LD_LIBRARY_PATH}"
export MAKEFLAGS=-j2
```

如果 `acados` 目录不存在，先检查：

```bash
ls -d /home/eisa/.local/acados
ls -d /home/eisa/.local/acados/lib
```

## 5. 获取 Autoware.universe

如果工控机还没有初始化 Autoware 工作区，可按官方常规方式准备 `src/` 后再执行编译。本文不重复列出完整官方 clone 清单，重点记录 WVCSC 工控机编译策略。

默认工作区目录：

```bash
cd /home/eisa/autoware
```

## 6. 编译 Autoware

推荐命令：

```bash
cd /home/eisa/autoware
source /opt/ros/humble/setup.bash

export CMAKE_PREFIX_PATH="/home/eisa/.local/acados:${CMAKE_PREFIX_PATH}"
export ACADOS_SOURCE_DIR="/home/eisa/.local/acados"
export LD_LIBRARY_PATH="/home/eisa/.local/acados/lib:${LD_LIBRARY_PATH}"
export MAKEFLAGS=-j2

colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

说明：

- `--executor sequential`：限制包级并发
- `MAKEFLAGS=-j2`：限制编译级并发
- `RelWithDebInfo`：兼顾调试与运行性能

## 7. 编译完成后验证

### 7.1 基础 source

```bash
source /home/eisa/autoware/install/setup.bash
```

### 7.2 验证 `autoware_launch`

```bash
ros2 pkg prefix autoware_launch
```

预期：

- 能输出 `/home/eisa/autoware/install/autoware_launch`

### 7.3 验证 `path_optimizer`

```bash
ros2 pkg prefix autoware_path_optimizer
```

如果这里正常，但运行时仍报动态库错误，再检查：

```bash
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep acados
```

### 7.4 验证接口与核心包

```bash
ros2 interface show autoware_system_msgs/msg/AutowareState
ros2 pkg prefix autoware_vehicle_msgs
ros2 pkg prefix autoware_perception_msgs
```

## 8. 编译 WVCSC 工作区

Autoware 编译成功后，再编译实车工作区：

```bash
cd /home/eisa/WVCSC_S2Z_UTB_ARM
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

## 9. 工控机注意事项

### 9.1 推荐在 `tmux` 或纯终端编译

原因：

- 避免 GUI 被长时间占满
- 编译过程中掉线也不影响任务继续

### 9.2 长编译时间是正常现象

在工控机上，Autoware 全量编译耗时明显长于普通开发机，这不是异常。

### 9.3 内存不足时考虑交换区

如果出现明显卡死、编译被系统杀掉、桌面响应极慢，可考虑增加 swap：

```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### 9.4 避免多窗口同时高负载

编译期间不要同时做这些事：

- 大量 RViz 点云显示
- 浏览器开很多标签页
- 多个 `colcon build` 并发

## 10. 与 WVCSC 正式运行的关系

只要下面三步都成功，后续就可以直接进入官方主入口运行：

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

然后执行：

```bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/eisa/autoware_map/maps/wvcsc_map1 \
  vehicle_model:=wvcsc_vehicle \
  sensor_model:=wvcsc_sensor_kit \
  data_path:=/home/eisa/autoware_data
```

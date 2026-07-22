# WVCSC 实机 Bringup

所有命令均需先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

## 1. 建图

```bash
ros2 launch wvcsc_bringup real_cartographer.launch.py
```

此命令复用已验证的底盘、LiDAR、IMU、EKF 链，启动 Cartographer 和
`wvcsc_bringup/rviz/real_cartographer.rviz`，不启动 C10、Nav2、机械臂或任务节点。完成建图后保存为
`${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.{yaml,pgm}`：

```bash
bash $(ros2 pkg prefix wvcsc_bringup)/share/wvcsc_bringup/scripts/save_corn_map.sh
```

脚本默认使用上述路径；也可将其他输出基名作为第一个参数传入。

## 2. 实机导航

一个命令启动底盘、LiDAR、IMU、EKF、map_server、AMCL、Nav2 和一个 RViz；不启动 C10：

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py
```

默认地图为 `${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml`。
使用其他地图时显式传入绝对路径：

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py \
  map:="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/other.yaml"
```

该启动链与已通过实机验证的
`my_navigation2/launch/wtb_navigation2_fdimu.launch.py` 使用相同的硬件链、
Nav2 参数和 `tf_buffer_size`；硬件子 RViz 被关闭，只启动复制到
`wvcsc_bringup/rviz/real_navigation.rviz` 的导航 RViz。完整任务才加载
`real_sensors.launch.py` 和 C10。Nav2 速度平滑器的最终输出直接发布到
`/cmd_vel`；不启动 `wvcsc_safety` 速度门控。

当前实车使用 Yesense IMU。首次部署或更换工控机时，先安装
`/dev/yesense_IMU` 稳定串口别名：

```bash
cd "${HOME}/WVCSC_S2Z_UTB_ARM/src/yesense_ros2"
sudo ./yesense_udev.sh
ls -l /dev/yesense_IMU
```

随后启动导航，确认 Yesense 节点日志显示串口打开成功，并确认
`ros2 topic hz /imu` 持续有数据。串口路径由
`yesense_std_ros2/config/yesense_config.yaml` 中的
`/dev/yesense_IMU` 管理，不要写死 `ttyUSB0` 或 `ttyACM0`。

旧版 `fdilink_ahrs` 代码和包仍保留，但已从默认启动链停用；只有回滚到旧 IMU
时才恢复对应的注释启动块，禁止两个驱动同时运行。

## 3. 逐树实测停靠点

AMCL稳定后，人工驾驶并停稳。卷尺必须从车体上标记的
`base_footprint`原点测量，`+X`为车头前方、`+Y`为车体左侧：

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file ~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml \
  --map "${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml" \
  --capture-home

ros2 run wvcsc_bringup capture_site_pose -- \
  --file ~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml \
  --map "${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml" \
  --target-id corn_01 \
  --tree-forward-m <前向实测值> --tree-left-m <左向实测值> \
  --spray-duration 5.0
```

重复目标默认拒绝覆盖，确认更新时增加`--update`。完成全部目标后验证：

```bash
ros2 run wvcsc_bringup validate_site_mission -- \
  --file ~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml \
  --map "${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml"
```

任务文件与地图YAML及图片SHA256绑定；地图改变后旧任务会被拒绝。
采点脚本会自行调用 AMCL 的 `/request_nomotion_update`，无需额外终端循环调用该
服务；当前只要求流程成功，采点质量门限临时放宽为位置/偏航散布 ≤ 1.00 m/rad、
AMCL 位置/偏航标准差 ≤ 1.00 m/rad。门限集中定义在
`wvcsc_bringup/wvcsc_bringup/site_mission.py`，修改后重新构建并 source 工作区。
定位链稳定后应恢复严格门限；放宽门限不代表当前定位精度满足最终工程验收。

## 4. 完整定位与作业

```bash
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  mission_source:=measured \
  mission_file:="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml" \
  map:="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml"
```

启动前检查会在任何硬件节点运行之前验证：

- C10 和 Alicia-M 设备路径；
- 地图文件和 ROS 包；
- 实测任务、地图哈希、停车区域与采样质量；
- 独立 YOLO Python 环境；
- `yolov8s_real.pt`: `detect`, `{0: tree}`；
- `yolov8s_seg_real.pt`: `segment`, `{0: disease_leaf}`。
- `wvcsc_c10_camera/config/c10_intrinsics.yaml`；
- `$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/c10_handeye.yaml`；
- `~/.ros/wvcsc_calibration/nozzle.yaml`。

实机权重需由用户放入
`wvcsc_rgb_vision/models/`。权重缺失或类别契约不匹配时，整个实机启动会
在硬件上电前失败，不会回退到仿真权重。

任务默认不自动开始。检查相机、定位和 `/mission/plan` 后启动：

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

实测任务加载后不会自动执行。先检查`/mission/plan`中的树根坐标和停靠位姿，
再调用 `/mission/start`。需要兼容原 UAV/Mock 路径时使用
`mission_source:=uav`。

## 5. 停止与恢复

当前实机链路按项目要求绕过 `wvcsc_safety`。停止任务时先取消任务或终止
launch，确认 `/cmd_vel` 已归零，再使用机械臂控制命令：

```bash
ros2 service call /mission/cancel std_srvs/srv/Trigger "{}"
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: resume}"
ros2 service call /mission/reset std_srvs/srv/Trigger "{}"
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

绕过软件安全门后，不再有 `/safety/controlled_abort`、自动持续零速或软件急停
兜底。实车运行必须保留可触达的底盘物理急停，并由操作员确认车辆完全停止。

## 6. 标定

内参（8x6 内角点，25 mm）：

```bash
ros2 launch wvcsc_c10_camera c10_intrinsics.launch.py
```

手眼标定（`DICT_5X5_250`, ID 1, 70 mm）使用三个终端：

```bash
# 终端一：硬件、MoveIt、C10、ArUco、easy_handeye2服务
ros2 launch wvcsc_calibration c10_handeye.launch.py

# 终端二：Alicia-M自适应自动采集；输入s或空Enter开始，q取消
ros2 run wvcsc_calibration auto_calibration_collector --ros-args \
  --params-file $(ros2 pkg prefix wvcsc_calibration)/share/\
wvcsc_calibration/config/auto_handeye_alicia.yaml

# 终端三：SPACE/h/r/x安全介入
ros2 run wvcsc_arm_task motion_control_keyboard
```

手眼标定 launch 不加载 `real_sensors.launch.py`：不会启动底盘、LiDAR、IMU、EKF
或 Nav2。车辆必须保持停稳，并保留可触达的物理急停。

默认手眼输出为
`$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/c10_handeye.yaml`。
自动采集会先根据 marker 相对 `alicia_base_link` 的配置生成安全初始观察位，
再清空上一轮服务端样本，逐候选执行碰撞IK、Jacobian条件数、关节余量和OMPL
门控，并以Park/Horaud/Tsai-Lenz共识及离群剔除后结果原子写入。

随后根据实际喷嘴安装填写并验证：

```bash
mkdir -p ~/.ros/wvcsc_calibration
cp $(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/\
nozzle.example.yaml ~/.ros/wvcsc_calibration/nozzle.yaml
```

`nozzle.yaml`描述`tool0→spray_nozzle_link`、固定工距`1.00±0.05m`和湿喷
落点微调`pixel_trim`。实机任务默认要求三份标定均存在；任一缺失时前置检查
会阻止任务栈启动，不会用名义外参自动喷洒。

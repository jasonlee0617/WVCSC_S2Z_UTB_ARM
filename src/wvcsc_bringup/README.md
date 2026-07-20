# WVCSC 实机 Bringup

`wvcsc_bringup` 将建图与定位作业分为两个互斥模式。所有命令均需先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
```

## 1. 建图

```bash
ros2 launch wvcsc_bringup system_real.launch.py mode:=mapping
```

此模式只启动 `my_cartographer` 已验证的底盘、LiDAR、IMU、EKF、
Cartographer 和 RViz 链路，不启动 Nav2、AMCL、机械臂或任务节点。

```bash
bash $(ros2 pkg prefix wvcsc_bringup)/share/wvcsc_bringup/scripts/save_corn_map.sh
```

## 2. 逐树实测停靠点

先只启动传感器、AMCL、Nav2与安全速度门控，不启动机械臂和任务闭环：

```bash
ros2 launch wvcsc_bringup system_real.launch.py \
  mode:=localization operation:=survey \
  map:=/home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml
```

AMCL稳定后，人工驾驶并停稳。卷尺必须从车体上标记的
`base_footprint`原点测量，`+X`为车头前方、`+Y`为车体左侧：

```bash
ros2 run wvcsc_bringup capture_site_pose -- \
  --file ~/.ros/wvcsc_sites/corn_site.yaml \
  --map /home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml \
  --capture-home

ros2 run wvcsc_bringup capture_site_pose -- \
  --file ~/.ros/wvcsc_sites/corn_site.yaml \
  --map /home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml \
  --target-id corn_01 \
  --tree-forward-m <前向实测值> --tree-left-m <左向实测值> \
  --spray-duration 5.0
```

重复目标默认拒绝覆盖，确认更新时增加`--update`。完成全部目标后验证：

```bash
ros2 run wvcsc_bringup validate_site_mission -- \
  --file ~/.ros/wvcsc_sites/corn_site.yaml \
  --map /home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml
```

任务文件与地图YAML及图片SHA256绑定；地图改变后旧任务会被拒绝。

## 3. 定位与作业

```bash
ros2 launch wvcsc_bringup system_real.launch.py \
  mode:=localization operation:=mission mission_source:=measured \
  mission_file:=~/.ros/wvcsc_sites/corn_site.yaml \
  map:=/home/robot/WVCSC_S2Z_UTB_ARM/src/my_navigation2/maps/map_new.yaml
```

启动前检查会在任何硬件节点运行之前验证：

- C10 和 Alicia-M 设备路径；
- 地图文件和 ROS 包；
- 实测任务、地图哈希、停车区域与采样质量；
- 独立 YOLO Python 环境；
- `yolov8s_real.pt`: `detect`, `{0: tree}`；
- `yolov8s_seg_real.pt`: `segment`, `{0: disease_leaf}`。
- `~/.ros/camera_info/c10.yaml`；
- `~/.ros/wvcsc_calibration/c10_handeye.yaml`；
- `~/.ros/wvcsc_calibration/nozzle.yaml`。

实机权重需由用户放入
`wvcsc_rgb_vision/models/`。权重缺失或类别契约不匹配时，整个实机启动会
在硬件上电前失败，不会回退到仿真权重。

任务默认不自动开始。相机正常、无安全锁定后：

```bash
ros2 service call /safety/set_autonomy_enabled \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

实测任务加载后不会自动执行。先检查`/mission/plan`中的树根坐标和停靠位姿，
再启用自动模式并调用`/mission/start`。需要兼容原UAV/Mock路径时使用
`mission_source:=uav`。

## 4. 受控中止与恢复

```bash
ros2 service call /safety/controlled_abort std_srvs/srv/Trigger "{}"
```

安全节点会立即取消任务、以 20 Hz 连续输出零速、停止机械臂，并在
底盘连续停稳 1 s 后请求 `reset -> HOME`。到达 HOME 后仍保持锁定：

```bash
ros2 service call /safety/reset std_srvs/srv/Trigger "{}"
ros2 topic pub --once /motion_control/command \
  std_msgs/msg/String "{data: resume}"
ros2 service call /safety/set_autonomy_enabled \
  std_srvs/srv/SetBool "{data: true}"
ros2 service call /mission/reset std_srvs/srv/Trigger "{}"
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

物理急停 `/safety/emergency_stop=true` 时，安全节点持续输出零速，机械臂
立即进入硬停止锁定；`reset`、HOME 和 `resume` 都会在机械臂最终执行边界被
拒绝。必须先解除物理急停，再调用 `/safety/controlled_abort`，由安全节点在
确认底盘停稳后执行受控 `reset -> HOME`，最后按上述顺序人工解锁并重启任务。

## 5. 标定

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

`x` 总是先发送机械臂 `stop`；完整实机系统还会调用
`/safety/controlled_abort`。独立标定没有启动底盘安全节点时，`x` 仍能可靠
停臂，但不会触发底盘恢复链。

默认手眼输出为 `~/.ros/wvcsc_calibration/c10_handeye.yaml`。自动采集会清空
上一轮服务端样本，逐候选执行碰撞IK、Jacobian条件数、关节余量和OMPL门控，
并以Park/Horaud/Tsai-Lenz共识及离群剔除后结果原子写入。

随后根据实际喷嘴安装填写并验证：

```bash
mkdir -p ~/.ros/wvcsc_calibration
cp $(ros2 pkg prefix wvcsc_calibration)/share/wvcsc_calibration/config/\
nozzle.example.yaml ~/.ros/wvcsc_calibration/nozzle.yaml
```

`nozzle.yaml`描述`tool0→spray_nozzle_link`、固定工距`1.00±0.05m`和湿喷
落点微调`pixel_trim`。实机任务默认要求三份标定均存在；任一缺失时前置检查
会阻止任务栈启动，不会用名义外参自动喷洒。

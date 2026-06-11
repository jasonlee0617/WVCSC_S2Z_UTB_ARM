# Autoware 启动排障与修复复盘

## 1. 问题背景

WVCSC 实车最终目标是直接使用官方命令启动：

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/WVCSC_S2Z_UTB_ARM/install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/eisa/autoware_map/maps/wvcsc_map1 \
  vehicle_model:=wvcsc_vehicle \
  sensor_model:=wvcsc_sensor_kit \
  data_path:=/home/eisa/autoware_data
```

在本轮修复前，虽然系统能部分启动，但存在四条关键问题链：

1. RViz 点击 `2D Pose Estimate` / `2D Goal` 后，终端出现 LLT 报错刷屏  
2. 路线能规划出来，但车辆不执行自动导航  
3. RViz 中 `Auto` 不可用  
4. 系统监控与 fail-safe 持续报错，影响控制链路

## 2. 根因汇总

### 2.1 LLT 刷屏

直接原因：

- IMU 协方差长期为零或近零
- `wtb_car` 发布的 `/car_odom` 协方差也过小
- EKF 融合 pose/twist 时数值条件过差
- 最终触发 LLT 相关数值退化报错

### 2.2 规划后不走车 / `Auto` 不可用

主根因：

- ROS 2 没有显式隔离 domain，默认落在公共 domain
- 同网段其他 Autoware 节点参与同一 DDS 图
- duplicated-node checker 报重复节点
- fail-safe / MRM 把 `vehicle_cmd_gate` 拉进 `Emergency`
- 所以轨迹虽然规划成功，但控制指令被门控为停车

### 2.3 交通灯感知缺失

WVCSC 当前没有真实交通灯识别链，但完整 Autoware 栈会检查：

- `/perception/traffic_light_recognition/traffic_signals`

这个 topic 长时间缺失会让系统监控长期报错，干扰自动模式可用性判断。

### 2.4 控制模式链路不完整

`wvcsc_vehicle_interface` 之前缺少：

- `/control/control_mode_request` 服务
- `/vehicle/status/control_mode` 状态链

结果是 Autoware 侧的“使能控制 / 自动接管”流程无法真正落到底盘接口。

## 3. 修复过程

### 3.1 `fdilink_ahrs_ROS2`

修改目标：

- 给 IMU 显式配置非零协方差
- 让 EKF 不再把 IMU 当成“无限可信”输入
- 调整 IMU `frame_id` 为当前运行链一致使用的 `base_link`

结果：

- IMU 输入更符合 Autoware / EKF 预期
- LLT 刷屏问题消失

### 3.2 `wtb_car_driver`

修改目标：

- 给 `/car_odom` 补齐非零协方差
- 减少底盘启动状态重复发布带来的噪声

结果：

- EKF 融合条件改善
- 底盘反馈更适合定位和状态链使用

### 3.3 `wvcsc_vehicle_interface`

修改目标：

- 新增 `/control/control_mode_request`
- 发布 `/vehicle/status/control_mode`
- 只有在 Autoware 控制模式下才接受 `/control/command/control_cmd`
- 控制超时后停车，但保持模式状态可追踪

结果：

- Autoware 能真正完成“接管控制 -> 下发控制命令 -> 底盘执行”这条链

### 3.4 `wvcsc_sensor_kit_launch`

修改目标：

- 增加 `traffic_light_heartbeat.py`
- 在没有真实交通灯识别的情况下，对 `/perception/traffic_light_recognition/traffic_signals` 做心跳占位

结果：

- 交通灯识别缺失不再持续放大为系统硬故障

### 3.5 环境钩子

修改目标：

- 自动补齐 `LD_LIBRARY_PATH` 中的 `~/.local/acados/lib`
- 自动设置 `ROS_DOMAIN_ID=88`

结果：

- `path_optimizer` 相关动态库可正常加载
- ROS 2 默认串域问题被固定规避

## 4. 验证证据

### 4.1 LLT 刷屏已消失

表现：

- 使用官方 `autoware.launch.xml` 直接启动后，不再出现持续 LLT 刷屏

### 4.2 duplicated nodes 消失

表现：

- 关键节点只剩单实例
- 不再出现跨机器混入同一 ROS 图的大面积重复节点

### 4.3 fail-safe 回归正常

表现：

- `/system/fail_safe/mrm_state` 回到 `state: 1 / behavior: 1`
- `vehicle_cmd_gate` 不再因为外部串域问题持续 `Emergency`

### 4.4 官方主入口可直接运行

表现：

- 不再需要 `wvcsc.launch.xml`
- 直接使用官方 `autoware.launch.xml` 即可启动整套运行链

## 5. 最终状态

当前已经达到：

- 官方启动命令可正常运行
- LLT 刷屏已消失
- 运行环境自动具备 `ROS_DOMAIN_ID=88`
- 控制模式链路已补齐
- 交通灯 topic 已有心跳占位
- `wvcsc_autoware_bringup` 不再作为正式主入口保留

## 6. 涉及的关键模块

- `fdilink_ahrs_ROS2`
- `wtb_car_driver`
- `wvcsc_vehicle_interface`
- `wvcsc_sensor_kit_launch`
- `wvcsc_vehicle_launch` 中的新环境钩子

## 7. 后续建议

1. 继续以官方 `autoware.launch.xml` 作为唯一主入口  
2. 把 `src/docs/` 作为唯一维护文档目录  
3. `wvcsc_autoware_bringup` 仅保留迁移壳层，待引用清理完成后删除  

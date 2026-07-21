# WVCSC 实车导航验收指南（无机械臂）

> 本文描述在**不启动机械臂**的情况下，独立验证小车 SLAM 建图、Nav2 导航、逐树停靠和坐标采集的完整流程。

## 1. 前置条件

- 小车底盘、LiDAR、IMU 硬件正常，所有线缆连接牢固
- 遥控器电量充足，急停开关可正常触发
- 已安装 `wvcsc_bringup` 包

---

## 阶段 1：建图

### 1.1 启动 Cartographer 建图

```bash
ros2 launch wvcsc_bringup real_cartographer.launch.py
```

启动流程：
1. 确认 RViz 窗口打开，能看到 LiDAR 扫描点（红色点云）和 Cartographer 子地图
2. **小车在原地静止至少 5 秒**，让 EKF 融合 IMU 和轮式里程计后收敛
3. 用遥控器**低速**（≤ 0.3 m/s）控制小车在作业区域内行驶一整圈
4. 行驶路径需覆盖所有玉米树所在的路段，确保 Cartographer 回环闭合（回到起点附近时地图自动对齐）
5. 观察 RViz 中地图不再有重影或漂移

### 1.2 保存地图

```bash
# 先结束当前轨迹
ros2 service call /write_state cartographer_ros_msgs/srv/FinishTrajectory \
  "{trajectory_id: 0}"

# 保存为 PGM + YAML
ros2 run nav2_map_server map_saver_cli \
  -f /home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/map/orchard
```

生成文件：
- `orchard.pgm` — 栅格地图图片
- `orchard.yaml` — 地图元数据（分辨率、原点）

**验收**：`orchard.yaml` 中的 `origin` 坐标正确（地图左下角在 map 坐标系中的位置），`resolution: 0.05`。

---

## 阶段 2：采点（导航模式 + 遥控器驱动）

### 2.1 启动导航

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py \
  map:=/home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/map/orchard.yaml
```

启动后确认：
- RViz 窗口中能看到已加载的地图（灰色占用栅格）
- `ros2 topic echo /amcl_pose` 有正常输出

### 2.2 初始化 AMCL 定位

在 RViz 中：
1. 点击顶部工具栏的 **`2D Pose Estimate`** 按钮
2. 在地图上小车实际所在位置点击，并拖拽箭头指向车头方向
3. 释放后 AMCL 粒子云（绿色箭头簇）应快速收敛到单簇
4. 如果粒子云发散或不收敛，重新点击 `2D Pose Estimate`

### 2.3 将小车移动到玉米树旁

**方式 A：RViz Nav2 Goal 导航（推荐）**
1. 点击顶部工具栏的 **`Navigation2 Goal`** 按钮
2. 在地图上点击目标位置，拖拽箭头指向期望的停靠航向
3. Nav2 自动规划路径并导航到目标

**方式 B：遥控器手动驾驶**
1. 遥控器低速控制小车到玉米树侧方
2. 车头朝向与道路平行（航向接近 0° 或 180°）
3. 观察 RViz 中 `base_footprint` 的位置确认已到达作业位置

### 2.4 确认停稳

在终端监控停稳状态：

```bash
ros2 topic echo /ekf_odom
```

确认 `twist.twist.linear.x` 和 `twist.twist.angular.z` 持续 1 秒均 ≤ 0.03（小车完全静止）。

同时确认 AMCL 协方差合格：

```bash
ros2 topic echo /amcl_pose
```

`pose.covariance[0]`（X 方差）和 `pose.covariance[7]`（Y 方差）的平方根 ≤ 0.08 m。

### 2.5 测量树到小车的距离

小车停稳后，用卷尺测量两个距离：

```
俯视图：

        ←── 卷尺横向距离 (tree-left-m) ──→
   ┌──────────────────────────────────────●  ← 玉米树根部 (tree_hint)
   │
   │
   ↑
   ◎ ← base_footprint (小车底盘投影中心)
   │
   └── 卷尺纵向距离 (tree-forward-m)，车头方向
```

| 参数 | 测量基准点 | 测量方向 | 工具 | 示例值 |
|------|-----------|---------|------|--------|
| `--tree-forward-m` | base_footprint 中心 | **车头朝向**（base_footprint +X） | 卷尺 | `0.0`（树在侧方） |
| `--tree-left-m` | base_footprint 中心 | **小车左侧**（+Y，右侧填**负数**） | 卷尺 | `1.50`（树在左侧 1.5m） |

**实测方法**：
1. 从 base_footprint 中心（小车底盘投影到地面的中心点）向车头方向拉卷尺，垂直树根部——读数为 `tree-forward-m`
2. 从同一中心点向小车左侧拉卷尺——读数为 `tree-left-m`（右侧为负，填 `-1.5` 等）
3. 如果树就在小车正侧面，`tree-forward-m` 填 `0.0`

### 2.6 执行采点

```bash
ros2 run wvcsc_bringup capture_site_pose \
  --map /home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/map/orchard.yaml \
  --target-id tree_01 \
  --tree-forward-m 0.0 \
  --tree-left-m 1.50
```

脚本执行流程：
1. 等待 AMCL 定位稳定 + EKF 停稳 1 秒
2. 连续采集 30 个样本（0.1 秒/次，共 3 秒），验证位置散布 ≤ 0.03 m
3. 调用 `tree_hint_from_offset()` 自动计算树根 map 坐标
4. 写入 `~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml`（自动创建或追加）

成功输出示例：
```
[SITE] captured tree_01 pose=(3.002,0.498,0.003) file=~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml
```

### 2.7 重复所有树

对每棵玉米树重复步骤 2.3~2.6：

```bash
# tree_01 (左侧)
ros2 run wvcsc_bringup capture_site_pose \
  --map .../orchard.yaml --target-id tree_01 \
  --tree-forward-m 0.0 --tree-left-m 1.50

# tree_02 (右侧)
ros2 run wvcsc_bringup capture_site_pose \
  --map .../orchard.yaml --target-id tree_02 \
  --tree-forward-m 0.1 --tree-left-m -1.55

# tree_03 (左侧)
ros2 run wvcsc_bringup capture_site_pose \
  --map .../orchard.yaml --target-id tree_03 \
  --tree-forward-m -0.1 --tree-left-m 1.48

# tree_04 (右侧)
ros2 run wvcsc_bringup capture_site_pose \
  --map .../orchard.yaml --target-id tree_04 \
  --tree-forward-m 0.0 --tree-left-m -1.52
```

### 2.8 验证 YAML

```bash
cat ~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml
```

预期格式：

```yaml
schema_version: 1
map_file: /home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/map/orchard.yaml
map_sha256: abc123...
mission:
  site_id: corn_site
  mission_id: corn_measured_001
  targets:
    - target_id: tree_01
      docking_pose: {x: 3.002, y: 0.498, yaw: 0.003}
      tree_hint: {x: 3.002, y: 1.998, z: 0.0}
      measured_tree_offset: {forward_m: 0.0, left_m: 1.5}
      spray_side: left
      spray_duration: 5.0
      capture_quality: {samples: 30, position_spread_m: 0.012, ...}
    - target_id: tree_02
      ...
```

**注意**：`docking_pose` 和 `tree_hint` 由脚本自动计算，**不需要手动填写**。

---

## 阶段 3：验收标准

| # | 检查项 | 标准 |
|---|--------|------|
| 1 | Cartographer 回环闭合 | 地图无重影、无断裂 |
| 2 | AMCL 初始定位 | 粒子云在 3 秒内收敛 |
| 3 | 每棵树停稳 | 1 秒内线速度 ≤ 0.03 m/s，角速度 ≤ 0.03 rad/s |
| 4 | AMCL 协方差 | 位置标准差 ≤ 0.08 m |
| 5 | 位置散布 | 30 个样本散布 ≤ 0.03 m |
| 6 | 重复性 | 同一棵树连续 3 次停靠误差 ≤ 0.12 m |
| 7 | corn_site.yaml | 4 棵树全部记录，格式校验通过 |

---

> 验收通过后，下一步启动完整任务：
> ```bash
> ros2 launch wvcsc_bringup real_system_mission.launch.py \
>   map:=/home/robot/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/map/orchard.yaml \
>   mission_file:=~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml
> ```
> 参考 [WVCSC_S2Z_UTB_ARM_Codex任务闭环实施方案](WVCSC_S2Z_UTB_ARM_Codex任务闭环实施方案.md)。

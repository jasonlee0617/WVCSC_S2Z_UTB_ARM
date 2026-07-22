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
0. 该启动不加载 C10；它复用已验证的底盘、LiDAR、IMU、EKF 硬件链和
   `wvcsc_bringup/rviz/real_cartographer.rviz`
1. 确认 RViz 窗口打开，能看到 LiDAR 扫描点（红色点云）和 Cartographer 子地图
2. **小车在原地静止至少 5 秒**，让 EKF 融合 IMU 和轮式里程计后收敛
3. 用遥控器**低速**（≤ 0.3 m/s）控制小车在作业区域内行驶一整圈
4. 行驶路径需覆盖所有玉米树所在的路段，确保 Cartographer 回环闭合（回到起点附近时地图自动对齐）
5. 观察 RViz 中地图不再有重影或漂移

### 1.2 保存地图

```bash
bash "$(ros2 pkg prefix wvcsc_bringup)/share/wvcsc_bringup/scripts/save_corn_map.sh"
```

脚本默认保存到
`${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard`。

生成文件：
- `orchard.pgm` — 栅格地图图片
- `orchard.yaml` — 地图元数据（分辨率、原点）

**验收**：`orchard.yaml` 中的 `origin` 坐标正确（地图左下角在 map 坐标系中的位置），`resolution: 0.05`。

---

## 阶段 2：采点（导航模式 + 遥控器驱动）

### 2.1 启动导航

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py
```

该命令直接启动底盘、LiDAR、IMU、EKF、AMCL、Nav2 和一个 RViz，不加载 C10，默认读取
`${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml`。Nav2 最终速度
直接发布到 `/cmd_vel`，不经过 `wvcsc_safety`。RViz 使用
`wvcsc_bringup/rviz/real_navigation.rviz`。

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

当前采点脚本使用调试阶段的宽松门限：位置标准差 ≤ 0.60 m、偏航标准差 ≤ 0.40 rad。
定位链稳定后，应再恢复为工程验收门限。

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
  --map "${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml" \
  --target-id tree_01 \
  --tree-forward-m 0.0 \
  --tree-left-m 1.50
```

脚本执行流程：
1. 自行调用 AMCL 的 `/request_nomotion_update`，等待 AHRS、AMCL 定位稳定 + EKF 停稳 1 秒
2. 连续采集 30 个样本（0.1 秒/次），当前调试阶段位置散布 ≤ 0.25 m、偏航散布 ≤ 0.25 rad
3. 调用 `tree_hint_from_offset()` 自动计算树根 map 坐标
4. 写入 `~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml`（自动创建或追加）

采点前必须确认 `/dev/yesense_IMU` 存在且 `ros2 topic hz /imu` 有持续输出；
AMCL 位置和偏航标准差均须不大于 `0.08`，不满足时脚本会拒绝写入。

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
map_file: <当前用户HOME>/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml
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

## 阶段 3：自动导航验收（无机械臂）

### 3.1 目的

用刚采集的 `corn_site.yaml` 中保存的 `docking_pose`，让小车依次自动导航到每棵玉米树的停靠位置。到达每个点后停留 2 秒，目视确认停靠精度后自动前往下一目标。

### 3.2 启动导航

```bash
ros2 launch wvcsc_bringup real_navigation.launch.py
```

在 RViz 中确认 AMCL 粒子云收敛（使用 `2D Pose Estimate` 初始化定位）。

### 3.3 执行顺序导航

```bash
ros2 run wvcsc_bringup nav_validate_sites.py \
  --file ~/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml \
  --pause-sec 2.0
```

**执行流程**：

```
[1/4] tree_01: navigating to (3.002, 0.498, 0.003)
[1/4] tree_01: arrived
[1/4] tree_01: pausing 2.0s...
[2/4] tree_02: navigating to (5.001, -0.502, 0.001)
[2/4] tree_02: arrived
[2/4] tree_02: pausing 2.0s...
[3/4] tree_03: ...
[4/4] tree_04: ... arrived
[4/4] tree_04: last target — done
[VALIDATE] all 4 targets completed successfully
```

关键行为：
- 逐目标调用 `/navigate_to_pose` Action，超时 120 秒
- Nav2 返回 `SUCCEEDED` 后原地等 `--pause-sec` 秒
- 任一目标失败 → 脚本报错退出，不继续后续目标
- 无需手动触发——全程自动顺序执行

### 3.4 验收细节

在每个 2 秒停留期间：
1. 观察 RViz 中 `base_footprint` 是否与地图上记录的停靠点重合
2. 目视或用卷尺确认实际位置误差 ≤ 0.12 m

---

## 阶段 4：验收标准

| # | 检查项 | 标准 |
|---|--------|------|
| 1 | Cartographer 回环闭合 | 地图无重影、无断裂 |
| 2 | AMCL 初始定位 | 粒子云在 3 秒内收敛 |
| 3 | 每棵树停稳 | 1 秒内线速度 ≤ 0.03 m/s，角速度 ≤ 0.03 rad/s |
| 4 | AMCL 协方差 | 调试阶段位置标准差 ≤ 0.60 m、偏航标准差 ≤ 0.40 rad |
| 5 | 位置散布 | 调试阶段 30 个样本位置/偏航散布 ≤ 0.25 m/0.25 rad |
| 6 | 重复性 | 同一棵树连续 3 次停靠误差 ≤ 0.12 m |
| 7 | corn_site.yaml | 4 棵树全部记录，格式校验通过 |
| 8 | 自动顺序导航 | 4 个目标全部 `SUCCEEDED`，无超时、无跳过 |

---

> 验收通过后，下一步启动完整任务：
> ```bash
> ros2 launch wvcsc_bringup real_system_mission.launch.py \
>   map:="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard.yaml" \
>   mission_file:="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/config/wvcsc_sites/corn_site.yaml"
> ```
> 参考 [WVCSC_S2Z_UTB_ARM_Codex任务闭环实施方案](WVCSC_S2Z_UTB_ARM_Codex任务闭环实施方案%20copy.md)。

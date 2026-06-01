# wvcsc_vehicle_launch

WVCSC 小车车身启动配置（Autoware 标准接口）。

## 文件

| 文件 | 说明 |
|------|------|
| `launch/vehicle.launch.xml` | 启动 robot_state_publisher，加载 URDF 模型 |
| `launch/vehicle_interface.launch.xml` | 启动底盘控制适配器节点 |
| `scripts/cleanup_autoware_sim.sh` | 清理 planning_simulator 残留进程 |

## 使用方法

被 `planning_simulator.launch.xml` 通过 `vehicle_model:=wvcsc_vehicle` 自动调用。
也可独立启动：`ros2 launch wvcsc_vehicle_launch vehicle.launch.xml`

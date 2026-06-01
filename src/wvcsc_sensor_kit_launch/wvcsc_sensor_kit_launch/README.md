# wvcsc_sensor_kit_launch

WVCSC 小车传感器套件启动配置（Autoware 标准接口）。

## 文件

| 文件 | 说明 |
|------|------|
| `launch/lidar.launch.xml` | LiDAR 驱动 + 3D→2D 点云转换 |
| `launch/imu.launch.xml` | IMU 驱动 |
| `launch/gnss.launch.xml` | GNSS 驱动（预留） |
| `launch/sensing.launch.xml` | 聚合所有传感器 + CAN 通信 |
| `launch/sensor_kit.launch.xml` | 感知 + 底盘 + 适配器 总启动 |
| `config/dummy_diag_publisher/sensor_kit.param.yaml` | 诊断发布器参数 |

## 使用方法

被 `planning_simulator.launch.xml` 通过 `sensor_model:=wvcsc_sensor_kit` 自动调用。
也可独立启动子模块：
- `ros2 launch wvcsc_sensor_kit_launch lidar.launch.xml`
- `ros2 launch wvcsc_sensor_kit_launch imu.launch.xml`
- `ros2 launch wvcsc_sensor_kit_launch sensing.launch.xml`

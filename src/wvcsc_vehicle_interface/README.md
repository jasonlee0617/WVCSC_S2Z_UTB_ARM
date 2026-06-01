# wvcsc_vehicle_interface

Autoware 控制指令 ↔ WTB 底盘适配器。

## 功能

- 订阅 Autoware 标准控制指令，转换为 `/cmd_vel` + `/run_static`
- 订阅 `/car_odom`，回传 Autoware 车辆状态话题

## 话题

| 订阅 | 发布 |
|------|------|
| `/control/command/control_cmd` | `/cmd_vel` |
| `/control/command/gear_cmd` | `/run_static` |
| `/car_odom` | `/vehicle/status/velocity_status` |
| | `/vehicle/status/steering_status` |
| | `/vehicle/status/control_mode` |
| | `/vehicle/status/gear_status` |

## 启动

由 `wvcsc_vehicle_launch/launch/vehicle_interface.launch.xml` 启动。

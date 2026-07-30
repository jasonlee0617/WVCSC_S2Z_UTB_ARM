# 介绍
imu驱动

# 运行
1. 安装 CH9102 IMU 的稳定串口别名
```
cd $HOME/WVCSC_S2Z_UTB_ARM/src/fdilink_ahrs_ROS2
sudo ./udev.sh
ls -l /dev/FDI_IMU_GNSS
```
该规则匹配 `1a86:55d4` 的 CH9102，并将其映射为
`/dev/FDI_IMU_GNSS`。不要在 AHRS 参数中写死 `/dev/my_robot`。
2.运行imu驱动
```
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```
3.订阅topic
```
ros2 topic echo /imu
```

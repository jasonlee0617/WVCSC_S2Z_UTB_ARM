# 介绍
imu驱动

# 运行
1. 串口改名，增加权限
```
./udev.sh
chmod 666 /dev/ttyUSB*
```
2.运行imu驱动
```
ros2 launch fdilink_ahrs ahrs_driver.launch.py
```
3.订阅topic
```
ros2 topic echo /imu
```
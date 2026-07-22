1. 测试环境说明
基于Ubuntu 22.04.4 LTS/Humble、Ubuntu 20.04.4 LTS/Foxy、Ubuntu 20.04.4 LTS/Rolling版本测试

2. 在工作区根目录下打开终端，先加载 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
```

3. set serial library path
Yesense 使用工作区中的 `serial` 包，不需要手工修改 `/etc/ld.so.conf`。

4. 生成msg消息头文件
在工作区根目录下执行：

```bash
colcon build --symlink-install --packages-up-to yesense_std_ros2
source install/setup.bash
```
（在更改msg消息格式时需重新编译）或着把该命令添加到build.sh脚本中

5. build package
- 在工作区根目录下执行 `colcon build --symlink-install --packages-up-to yesense_std_ros2`
- 在Foxy版本下会编译出错，需要将yesense_node.cpp源文件中把407行代码注释掉，使用404行的代码
 
6. run
- 默认配置为：设备 `/dev/yesense_IMU`、波特率 `460800`，使用 ROS 串口驱动。
- 先确认 udev 别名存在：`ls -l /dev/yesense_IMU`。
- 然后执行：

```bash
source install/setup.bash
ros2 launch yesense_std_ros2 yesense_node.launch.py
```

不要使用 `sudo chmod 777` 临时修改串口权限；应修正 udev 规则并重新加载。
如需修改设备端口或波特率，修改
`yesense_ros2/yesense_std_ros2/config/yesense_config.yaml` 后重新编译；
`driver_type` 支持 `ros_serial` 和 `linux_serial`，波特率支持
`9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600`。

标准 ROS IMU 数据位于 `/imu`；Yesense 扩展数据默认位于
`/imu_data_extend`。查看标准数据：

```bash
ros2 topic echo /imu --once
```

7. 支持的topic
### ros 内置 topic

/sensor_msgs/msg/Imu            : 加速度、角速度、四元数
/visualization_msgs/msg/Marker  : 姿态，定位，形状数据（用于 rviz 可视化） 
/geometry_msgs/msg/Pose         : 传感器姿态数据 

### yesense 扩展 topic
/imu_data       ：imu数据，包含tid、加速度、角速度、传感器温度、采样时间戳
/sensor_10axis  : 10轴数据，包含tid、加速度、角速度、传感器温度、采样时间戳、磁场强度(原始数据和归一化数据)、气压（保留）

/euler_only     : 欧拉角数据，包括tid、横滚角、俯仰角、航向角
/robot_lord     : 机器人LORD格式数据，包括tid、加速度、角速度、四元数
/att_min_vru    : 推荐的姿态传感器VRU模式最小数据，包含tid、加速度、角速度、传感器温度、采样时间戳、欧拉角
/att_min_ahrs   : 推荐的姿态传感器AHRS模式最小数据，包含tid、加速度、角速度、磁场强度（原始数据和归一化数据）、传感器温度、采样时间戳、欧拉角
/att_all        : 姿态产品所有数据，包含tid、加速度、角速度、磁场强度（原始数据和归一化数据）、传感器温度、采样时间戳、欧拉角、四元数

/pos_only       : 位置数据，包括tid、经纬高、组合状态
/nav_min        : 推荐的组合导航产品的最小数据，包含tid、欧拉角、经纬高、组合状态
/nav_min_utc    : 推荐的组合导航产品的带UTC时间的最小数据，包含tid、欧拉角、经纬高、UTC时间、组合状态
/nav_all        : 组合导航所有的数据，包含tid、加速度、角速度、欧拉角、四元数、传感器温度、位置、组合状态、enu速度、utc时间、气压（保留）

8. 订阅者示例
- 在项目根目录下打开终端，输入 `ros2 run yesense_std_ros2 yesense_node_subscriber` 并执行（如果使用了新的终端，请先执行 `. install/setup.bash`命令）   

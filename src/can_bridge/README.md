# CAN通信驱动节点
功能一：使用CAN分析仪，接收can数据转换为ros2 topic 发布
功能儿：接收ros2 topic数据，转换为CAN数据发布到CAN总线上


# 消息订阅与发布
## 订阅
'''
/can_rx_1 : 
    说明：CAN 通道1消息话题，发送到通道1 CAN总线
    消息类型：can_msgs/msg/Frame
/can_rx_2 : 
    说明：CAN 通道2消息话题，发送到通道2 CAN总线
    消息类型：can_msgs/msg/Frame
'''
## 发布
'''
/can_tx_1 : 
    说明：接收CAN总线通道一数据，发送到ROS2
    消息类型：can_msgs/msg/Frame
/can_tx_2 : 
    说明：接收CAN总线通道二数据，发送到ROS2
    消息类型：can_msgs/msg/Frame
'''

# 运行
```
source install/setup.sh
ros2 launch can_bridge can_bridge.launch.py
```
# 配置
## CAN分析仪权限设置
创建文件
```
sudo vi /etc/udev/rules.d/99-myusb.rules 
```
写入
```
ACTION=="add",SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", GROUP="users", MODE="0777" 
```

# 环境安装
sudo apt-get install ros-$ROS_DISTRO-can-msgs
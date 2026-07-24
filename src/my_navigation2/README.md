# navigation2 导航

实车验收使用 `wvcsc_bringup real_navigation.launch.py`，默认加载
`param/wtb_nav2_params.yaml`。该参数中的 footprint 与 `wtb_car.xacro` 的
1.45 m × 0.75 m 车体及 ±0.43 m 轮距碰撞包络一致；局部/全局膨胀半径分别为
0.30 m / 0.35 m，避免原先 1.5 m / 0.8 m 膨胀造成整条通道被判高代价。

降低膨胀只减少额外代价缓冲，不关闭 footprint 碰撞检查。现场仍应先以低速
验证车体和轮胎不会擦碰障碍物，再逐步提高速度。

# 环境安装
```
sudo apt-get install ros-humble-nav2*
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2*
```


# 消息订阅与发布
## 订阅
'''

'''
## 发布
'''

'''

# 运行 
```
source install/setup.sh
运行导航功能
ros2 launch my_navigation2 eisa_navigation2.launch.py

```

# cartographer 2D建图

使用cartographer进行2D建图


# 环境安装
```
sudo apt-get install ros-humble-cartographer
sudo apt install ros-$ROS_DISTRO-cartographer
sudo apt install ros-$ROS_DISTRO-cartographer-ros

```
# 配置
'''
tracking_frame = "base_footprint", 	
如果只用激光，用激光的。imu+激光用imu，因为imu发布频率高

published_frame = "base_footprint", 	-- "odom",
cartographer发布的tf树指向published_frame，不是cartographer提供的，一般为底盘的frame_id,urdf文件的底盘link_name

MAP_BUILDER.use_trajectory_builder_2d = true    --2d建图minibot_type

MAP_BUILDER.num_background_threads = 4    --后端线条数，越大实时性越好

POSE_GRAPH.optimize_every_n_nodes = 50   -- node个优化一次
'''

# 使用lidar建图
```
ros2 launch turn_on_robot turn_on_robot.launch.py
ros2 launch lslidar_driver lslidar_cx_launch.py 
ros2 launch my_cartographer lidar_cartographer.launch.py 
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser_link 


ros2 launch my_cartographer lidar_cartographerALL.launch.py

```


# 使用lidar+里程计建图
```
ros2 launch turn_on_robot turn_on_robot.launch.py
ros2 launch lslidar_driver lslidar_cx_launch.py
ros2 launch my_cartographer cartographer.launch.py  

ros2 launch my_cartographer cartographerAll2.launch.py

```

# 保存地图

```
（指定地图保存路径-f <map_dir>/<map_name>）
ros2 run nav2_map_server map_saver_cli -f map 

("filename: '${PWD}/map.pbstream' : 保存地图路径，当前路径)
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "filename: '${PWD}/map.pbstream'"


```

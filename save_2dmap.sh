source install/setup.sh

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# 拼接地图文件的绝对路径

#rosrun map_server map_saver -f "$SCRIPT_DIR/src/my_navigation2/maps/map_new"
#ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "filename: '${PWD}/map.pbstream'"
#ros2 run nav2_map_server map_saver_cli -f "$SCRIPT_DIR/src/my_navigation2/maps/map_new"
# ros2 run nav2_map_server map_saver_cli -f "$SCRIPT_DIR/install/my_navigation2/share/my_navigation2/maps/map_new"
ros2 run nav2_map_server map_saver_cli -f "$HOME/ros_maps/map"
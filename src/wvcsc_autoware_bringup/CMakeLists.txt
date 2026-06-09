cmake_minimum_required(VERSION 3.8)
project(wvcsc_autoware_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_xyzirt_to_xyzirc_node
  src/pointcloud_xyzirt_to_xyzirc_node.cpp
)
ament_target_dependencies(pointcloud_xyzirt_to_xyzirc_node
  rclcpp
  sensor_msgs
)

install(DIRECTORY launch config docs rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_xyzirt_to_xyzirc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pgm_to_fake_pcd.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

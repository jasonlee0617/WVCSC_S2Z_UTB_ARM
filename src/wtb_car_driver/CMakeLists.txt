cmake_minimum_required(VERSION 3.8)
project(wtb_car_driver)
# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# 查找 ROS2 消息生成工具
find_package(rosidl_default_generators REQUIRED)
find_package(can_msgs REQUIRED)


# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarMsg.msg"
  DEPENDENCIES std_msgs
)


# 包含目录
include_directories(
  include
  lib
)

# 添加库目录
link_directories(
  lib
)

# 将源代码文件编译成可执行程序

add_executable(wtb_car src/wtb_car.cpp)
add_executable(wtb_car_only src/wtb_car_only.cpp)



# 获取系统架构
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")

# 根据架构设置不同的链接库
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(wtb_car controlcan)
  target_link_libraries(wtb_car_only controlcan)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(wtb_car controlcan_arm)
  target_link_libraries(wtb_car_only controlcan_arm)

  message(STATUS "Building for ARM architecture: ${CMAKE_SYSTEM_PROCESSOR}")
else()
  message(STATUS "Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 添加ROS2依赖

ament_target_dependencies(wtb_car 
  rclcpp 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
  sensor_msgs 
  nav2_msgs
  nav_msgs
  can_msgs
)

ament_target_dependencies(wtb_car_only 
  rclcpp 
  std_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
  sensor_msgs 
  nav2_msgs
  nav_msgs
  can_msgs
)




# 链接自定义消息

rosidl_target_interfaces(wtb_car 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# 安装可执行文件
install(TARGETS 
  wtb_car
  wtb_car_only 
  DESTINATION lib/${PROJECT_NAME})

# 安装库文件
install(
  DIRECTORY lib/  
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*")


# 安装资源文件
install(DIRECTORY
  launch
  rviz
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()



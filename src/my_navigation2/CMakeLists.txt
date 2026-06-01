cmake_minimum_required(VERSION 3.5)
project(my_navigation2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


install(
  DIRECTORY launch param maps rviz ros_maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(PROGRAMS scripts/trafficLight_nav2.py  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/trafficLightControl.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/qt_nav2.py DESTINATION lib/${PROJECT_NAME})
# install(PROGRAMS scripts/test.py DESTINATION lib/${PROJECT_NAME})

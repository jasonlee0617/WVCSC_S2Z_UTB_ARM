cmake_minimum_required(VERSION 3.8)
project(trajectory_retime_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RetimeTrajectory.srv"
  DEPENDENCIES trajectory_msgs
)

add_executable(retime_server src/retime_server.cpp)
target_compile_features(retime_server PUBLIC cxx_std_17)
target_include_directories(retime_server PRIVATE include)
ament_target_dependencies(retime_server
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(retime_server "${cpp_typesupport_target}")

install(TARGETS retime_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_validation test/test_validation.cpp)
  target_compile_features(test_validation PUBLIC cxx_std_17)
  target_include_directories(test_validation PRIVATE include)
endif()

ament_package()

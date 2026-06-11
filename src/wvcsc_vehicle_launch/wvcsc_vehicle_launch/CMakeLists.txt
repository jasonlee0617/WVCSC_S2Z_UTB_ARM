cmake_minimum_required(VERSION 3.8)
project(wvcsc_vehicle_launch)

find_package(ament_cmake REQUIRED)

set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_acados_library_path
  "prepend-non-duplicate;LD_LIBRARY_PATH;/home/eisa/.local/acados/lib")
set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_ros_domain_id
  "set-if-unset;ROS_DOMAIN_ID;88")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/acados_library_path.sh.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ros_domain_id.sh.in"
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/cleanup_autoware_sim.sh DESTINATION lib/${PROJECT_NAME})

ament_package()

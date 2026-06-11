cmake_minimum_required(VERSION 3.8)
project(wvcsc_sensor_kit_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/traffic_light_heartbeat.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

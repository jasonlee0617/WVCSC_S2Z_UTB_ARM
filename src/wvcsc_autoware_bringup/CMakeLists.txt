cmake_minimum_required(VERSION 3.8)
project(wvcsc_autoware_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

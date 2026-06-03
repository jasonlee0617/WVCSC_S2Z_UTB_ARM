cmake_minimum_required(VERSION 3.8)
project(wvcsc_autoware_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pgm_to_fake_pcd.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

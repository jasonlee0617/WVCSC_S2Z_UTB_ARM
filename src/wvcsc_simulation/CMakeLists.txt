cmake_minimum_required(VERSION 3.8)
project(wvcsc_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps models worlds rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ackermann_sim.py DESTINATION lib/${PROJECT_NAME})

ament_package()

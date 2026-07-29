# 中文说明：Gazebo、Nav2、机械臂和仿真辅助脚本的构建/安装入口。
# 资源安装路径必须使用 ament 的 Python 安装变量，避免只在单一 Python 版本下可用。
cmake_minimum_required(VERSION 3.8)
project(wvcsc_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(wvcsc_spray_visual_plugin SHARED src/spray_visual_plugin.cpp)
target_include_directories(wvcsc_spray_visual_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(wvcsc_spray_visual_plugin
  ${GAZEBO_LIBRARIES}
  Qt5::Core
  Qt5::Widgets)
ament_target_dependencies(wvcsc_spray_visual_plugin gazebo_ros rclcpp std_msgs)

ament_python_install_package(${PROJECT_NAME})

# data_acquisition is imported as wvcsc_simulation.data_acquisition in launch files.
# Install it beside the package using the active Python install directory rather
# than a distribution-specific Python version.
install(
  DIRECTORY data_acquisition
  DESTINATION ${PYTHON_INSTALL_DIR}/wvcsc_simulation
  PATTERN "scripts" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY config data_acquisition launch maps models worlds rviz DESTINATION share/${PROJECT_NAME})
install(TARGETS wvcsc_spray_visual_plugin LIBRARY DESTINATION lib)
install(PROGRAMS
  scripts/ackermann_sim.py
  wvcsc_simulation/sim_relay.py
  data_acquisition/scripts/capture_yolo_seed.py
  data_acquisition/scripts/capture_yolo_seed_direct.py
  data_acquisition/scripts/validate_yolo_seed.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    test_orchard_assets test/test_orchard_assets.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_yolo_seed_dataset test/test_yolo_seed_dataset.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_system_sim_launch test/test_system_sim_launch.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_ackermann_math test/test_ackermann_math.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_sim_relay test/test_sim_relay.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_add_pytest_test(
    test_calibration_sim_launch test/test_calibration_sim_launch.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()

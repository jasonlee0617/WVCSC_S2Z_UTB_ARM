# 中文说明：bringup 包的构建与安装入口，负责安装启动文件、Qt 脚本和运行配置。
# 本文件不实现任务逻辑；配置的实际含义由各 launch 和节点负责。
# 修改依赖或安装路径时必须保持实机启动链与仿真/测试入口可发现。
cmake_minimum_required(VERSION 3.8)
project(wvcsc_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/preflight_check.py
  scripts/save_corn_map.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS
  scripts/fake_arm_spray_action.py
  scripts/nav2_qt.py
  scripts/arm_spray_test_qt.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The host has an incompatible user-installed pytest plugin.  Keep package
  # tests hermetic so CMake can discover and execute the system pytest.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(test_real_bringup test/test_real_bringup.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_path_defaults test/test_path_defaults.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_nav2_qt_model test/test_nav2_qt_model.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_qt_image_viewer test/test_qt_image_viewer.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_arm_spray_test_qt test/test_arm_spray_test_qt.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()

ament_package()

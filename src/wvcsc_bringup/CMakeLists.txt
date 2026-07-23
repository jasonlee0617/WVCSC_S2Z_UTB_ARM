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
  scripts/capture_site_pose.py
  scripts/migrate_site_mission.py
  scripts/arm_spray_once.py
  scripts/field_route_manager.py
  scripts/load_site_mission.py
  scripts/nav_validate_sites.py
  scripts/validate_field_route.py
  scripts/validate_site_mission.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/capture_site_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME capture_site_pose)
install(PROGRAMS scripts/arm_spray_once.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_spray_once)
install(PROGRAMS scripts/migrate_site_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME migrate_site_mission)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The host has an incompatible user-installed pytest plugin.  Keep package
  # tests hermetic so CMake can discover and execute the system pytest.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(test_real_bringup test/test_real_bringup.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_capture_site_pose test/test_capture_site_pose.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_site_mission test/test_site_mission.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
  ament_add_pytest_test(test_field_route test/test_field_route.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1")
endif()

ament_package()

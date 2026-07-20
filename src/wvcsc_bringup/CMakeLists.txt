cmake_minimum_required(VERSION 3.8)
project(wvcsc_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/preflight_check.py
  scripts/save_corn_map.sh
  DESTINATION share/${PROJECT_NAME}/scripts)
install(PROGRAMS
  scripts/capture_site_pose
  scripts/load_site_mission
  scripts/validate_site_mission
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_real_bringup test/test_real_bringup.py)
  ament_add_pytest_test(test_site_mission test/test_site_mission.py)
endif()

ament_package()

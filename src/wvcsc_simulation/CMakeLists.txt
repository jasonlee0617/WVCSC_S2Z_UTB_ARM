cmake_minimum_required(VERSION 3.8)
project(wvcsc_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch maps models worlds rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/ackermann_sim.py
  scripts/capture_yolo_seed.py
  scripts/capture_yolo_seed_direct.py
  scripts/replace_yolo_seed_datasets.py
  scripts/validate_yolo_seed.py
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
endif()

ament_package()

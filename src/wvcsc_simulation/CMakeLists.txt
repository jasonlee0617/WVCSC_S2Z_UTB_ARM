cmake_minimum_required(VERSION 3.8)
project(wvcsc_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# data_acquisition is imported as wvcsc_simulation.data_acquisition in launch files.
# It must live alongside the wvcsc_simulation package (installed by ament_python_install_package).
install(
  DIRECTORY data_acquisition
  DESTINATION local/lib/python3.10/dist-packages/wvcsc_simulation
  PATTERN "scripts" EXCLUDE
  PATTERN "__pycache__" EXCLUDE)

install(DIRECTORY config data_acquisition launch maps models worlds rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/ackermann_sim.py
  data_acquisition/scripts/capture_yolo_seed.py
  data_acquisition/scripts/capture_yolo_seed_direct.py
  data_acquisition/scripts/replace_yolo_seed_datasets.py
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
  )
endif()

ament_package()

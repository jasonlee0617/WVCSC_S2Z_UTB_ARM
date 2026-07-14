cmake_minimum_required(VERSION 3.8)
project(alicia_m_descriptions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz config files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

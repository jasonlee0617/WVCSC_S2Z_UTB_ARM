cmake_minimum_required(VERSION 3.8)
project(can_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

message("*********************************************************************************************")


# 检测系统架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(ARCH_DIR "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i386|i686")
    set(ARCH_DIR "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ARCH_DIR "arm64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# 构建库文件路径
set(CONTROL_CAN_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH_DIR}")
set(CONTROL_CAN_LIB "${CONTROL_CAN_LIB_DIR}/libcontrolcan.so")

message(${CONTROL_CAN_LIB})

# 检查库文件是否存在
if(NOT EXISTS ${CONTROL_CAN_LIB})
    message(FATAL_ERROR "Library file ${CONTROL_CAN_LIB} not found.")
endif()


add_executable(can_bridge_node src/can_bridge_node.cpp)
ament_target_dependencies(can_bridge_node rclcpp can_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)


# 链接库文件
target_link_libraries(can_bridge_node ${CONTROL_CAN_LIB})


# 安装库文件
install(FILES ${CONTROL_CAN_LIB}
        DESTINATION lib/${PROJECT_NAME})

install(FILES  ${CONTROL_CAN_LIB}
DESTINATION lib/
)

# 安装可执行文件
install(TARGETS can_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY lib/                       # 发现你项目中的lib中所有的文件
    DESTINATION lib/${PROJECT_NAME}      # 拷贝到install目录中
  )

install(TARGETS can_bridge_node 
    RUNTIME DESTINATION lib/${PROJECT_NAME}    # 程序运行的时候调用install中的路径
  )

  
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)





ament_package()

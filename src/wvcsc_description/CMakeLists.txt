# 中文说明：机器人模型包的构建与 URDF/Xacro 安装入口。
# 本文件只安装模型、控制器和描述资源，不改变车辆/机械臂几何参数。
cmake_minimum_required(VERSION 3.8)
project(wvcsc_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config licenses meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()

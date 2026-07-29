# 中文说明：WVCSC ROS 2 消息、服务和 Action 的接口生成入口。
# 字段顺序、类型和常量值属于 DDS wire contract；这里只维护生成依赖与安装声明。
cmake_minimum_required(VERSION 3.8)
project(wvcsc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionPointPlan.msg"
  "msg/MissionPointStatus.msg"
  "msg/MissionPlan.msg"
  "msg/MissionStatus.msg"
  "msg/ManualMissionPoint.msg"
  "msg/Target2D.msg"
  "srv/ComputeSprayAim.srv"
  "srv/LoadManualMission.srv"
  "srv/SetRelay.srv"
  "action/AlignTarget.action"
  "action/ExecuteSpray.action"
  "action/Spray.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

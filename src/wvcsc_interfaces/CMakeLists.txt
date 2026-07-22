cmake_minimum_required(VERSION 3.8)
project(wvcsc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionTargetPlan.msg"
  "msg/MissionTargetStatus.msg"
  "msg/MissionPlan.msg"
  "msg/MissionStatus.msg"
  "msg/ManualMissionTarget.msg"
  "msg/Target2D.msg"
  "srv/ComputeSprayAim.srv"
  "srv/LoadManualMission.srv"
  "action/AlignTarget.action"
  "action/ExecuteSpray.action"
  "action/Spray.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

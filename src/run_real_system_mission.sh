#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/WVCSC_S2Z_UTB_ARM/install/setup.bash"
set -u
ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python"

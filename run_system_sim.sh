#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
# ROS setup scripts read this optional variable without a default while
# nounset is enabled.
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${COLCON_TRACE:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
: "${AMENT_PREFIX_PATH:=}"
: "${COLCON_PREFIX_PATH:=}"
: "${PYTHONPATH:=}"
: "${LD_LIBRARY_PATH:=}"
source /opt/ros/humble/setup.bash
source install/setup.bash

exec ros2 launch wvcsc_simulation system_sim.launch.py \
  perception_mode:=yolo \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python" \
  "$@"

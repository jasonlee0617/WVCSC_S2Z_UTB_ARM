#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${workspace_dir}"
source install/setup.bash
set -u
if [[ "${1:-}" == "--show-args" ]]; then
  exec ros2 launch --show-args wvcsc_bringup real_system_mission.launch.py
fi
exec ros2 launch wvcsc_bringup real_system_mission.launch.py \
  yolo_python_executable:="${HOME}/venvs/wvcsc_yolo_ros/bin/python" "$@"

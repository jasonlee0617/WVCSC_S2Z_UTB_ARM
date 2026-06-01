#!/usr/bin/env bash
#
# cleanup_autoware_sim.sh
#
# List or clean stale Autoware planning-simulator processes in the Docker
# container. By default this script only prints matched processes. Pass --kill
# to terminate stale ROS processes after accidentally pressing Ctrl-Z.
#
# This script intentionally does not kill rviz2 or any display-server process.
# If RViz freezes the desktop, handle the graphics stack separately.

set -euo pipefail

current_user="$(id -un)"
mode="${1:---list}"

if [[ "${mode}" != "--list" && "${mode}" != "--kill" ]]; then
  echo "Usage: $0 [--list|--kill]" >&2
  exit 2
fi

match_regex='ros2 launch autoware_launch planning_simulator.launch.xml|/opt/ros/humble/lib/rclcpp_components/component_container|/opt/ros/humble/lib/rclcpp_components/component_container_mt|robot_state_publisher|planning_evaluator|control_evaluator|routing_adaptor_node|initial_pose_adaptor_node|goal_pose_visualizer|dummy_diag_publisher'

echo "[cleanup] Current user: ${current_user}"
echo "[cleanup] Matched ROS processes:"
pgrep -afu "${current_user}" "${match_regex}" || true

if [[ "${mode}" == "--list" ]]; then
  echo "[cleanup] List-only mode. Re-run with --kill to terminate these processes."
  exit 0
fi

patterns=(
  'ros2 launch autoware_launch planning_simulator.launch.xml'
  '/opt/ros/humble/lib/rclcpp_components/component_container'
  '/opt/ros/humble/lib/rclcpp_components/component_container_mt'
  'robot_state_publisher'
  'planning_evaluator'
  'control_evaluator'
  'routing_adaptor_node'
  'initial_pose_adaptor_node'
  'goal_pose_visualizer'
  'dummy_diag_publisher'
)

for pattern in "${patterns[@]}"; do
  pkill -TERM -u "${current_user}" -f "${pattern}" 2>/dev/null || true
done

sleep 2

for pattern in "${patterns[@]}"; do
  pkill -KILL -u "${current_user}" -f "${pattern}" 2>/dev/null || true
done

ros2 daemon stop >/dev/null 2>&1 || true

echo "[cleanup] Remaining ROS-related processes:"
pgrep -afu "${current_user}" "${match_regex}" || true
echo "[cleanup] Done. Start planning_simulator again from a fresh shell state."

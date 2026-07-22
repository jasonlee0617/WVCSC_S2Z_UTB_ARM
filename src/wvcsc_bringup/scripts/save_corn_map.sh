#!/usr/bin/env bash
set -eo pipefail
# Usage: ./save_corn_map.sh [absolute_or_relative_output_basename]
# Default output matches real_navigation.launch.py's localization map.
source /opt/ros/humble/setup.bash
source "$HOME/WVCSC_S2Z_UTB_ARM/install/setup.bash"
set -u
output="${1:-${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard}"
mkdir -p "$(dirname "$output")"
ros2 run nav2_map_server map_saver_cli -f "$output"
echo "Map saved to ${output}.yaml"

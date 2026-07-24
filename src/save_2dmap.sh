#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/WVCSC_S2Z_UTB_ARM/install/setup.bash"
set -u

if [[ $# -ge 1 ]]; then
  output="$1"
else
  timestamp="$(date +%Y%m%d_%H%M%S)"
  output="${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/map_${timestamp}/orchard"
fi
mkdir -p "$(dirname "${output}")"
ros2 run nav2_map_server map_saver_cli -f "${output}"

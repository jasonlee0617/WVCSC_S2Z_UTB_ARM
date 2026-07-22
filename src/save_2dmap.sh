#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/WVCSC_S2Z_UTB_ARM/install/setup.bash"
set -u

output="${1:-${HOME}/WVCSC_S2Z_UTB_ARM/src/wvcsc_bringup/maps/orchard}"
mkdir -p "$(dirname "${output}")"
ros2 run nav2_map_server map_saver_cli -f "${output}"

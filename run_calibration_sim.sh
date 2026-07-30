#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/WVCSC_S2Z_UTB_ARM/install/setup.bash"
set -u
ros2 launch wvcsc_simulation calibration_sim.launch.py 


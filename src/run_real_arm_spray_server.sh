#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"

exec ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  use_qt_gui:=false "$@"

#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

# Qt 是默认的单臂喷洒入口；动作判定与编排仍由 SprayTask 负责。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="${script_dir}"

cd "${workspace_dir}"
source "${workspace_dir}/install/setup.bash"
set -u

exec ros2 launch wvcsc_bringup real_arm_spray_test.launch.py "$@"

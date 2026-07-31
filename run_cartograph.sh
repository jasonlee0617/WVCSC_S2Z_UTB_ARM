#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${workspace_dir}"

source install/setup.bash
set -u

exec ros2 launch wvcsc_bringup real_cartographer.launch.py "$@"

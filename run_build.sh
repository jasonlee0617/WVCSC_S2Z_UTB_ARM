#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${workspace_dir}"
exec colcon build --symlink-install "$@"

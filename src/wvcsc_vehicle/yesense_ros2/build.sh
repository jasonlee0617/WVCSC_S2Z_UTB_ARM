#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if ! REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null)"; then
  REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
fi
cd "$REPO_ROOT"

source /opt/ros/humble/setup.bash
echo "Building Yesense IMU from $REPO_ROOT"

# Build the complete Yesense dependency chain:
# serial -> yesense_interface -> yesense_std_ros2.
colcon build --symlink-install --packages-up-to yesense_std_ros2

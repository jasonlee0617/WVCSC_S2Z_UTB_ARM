#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash

workspace_root="${WVCSC_WORKSPACE:-$HOME/WVCSC_S2Z_UTB_ARM}"
workspace_setup="$workspace_root/install/setup.bash"
if [[ ! -f "$workspace_setup" ]]; then
  echo "Workspace setup not found: $workspace_setup" >&2
  exit 1
fi
source "$workspace_setup"
set -u

bag_dir="${WVCSC_BAG_DIR:-$HOME/bags/wvcsc}"
mkdir -p "$bag_dir"

exec ros2 bag record --include-hidden-topics -o "$bag_dir/wvcsc_servo_$(date +%Y%m%d_%H%M)" \
  /clock \
  /vision/target \
  /vision/tree_detections \
  /vision/fruit_detections \
  /vision/selected_target_id \
  /vision/inference_mode \
  /vision/visual_servo_debug \
  /servo_node/delta_twist_cmds \
  /servo_node/status \
  /servo_node/collision_velocity_scale \
  /joint_states \
  /arm_controller/joint_trajectory \
  /camera/camera/color/camera_info \
  /tf \
  /tf_static \
  /arm/observation_debug \
  /vision/perception_debug \
  /mission/status \
  /vision/align_target/_action/status \
  /vision/align_target/_action/feedback \
  /arm/execute_spray/_action/status \
  /arm/execute_spray/_action/feedback \
  /spray/execute/_action/status \
  /spray/execute/_action/feedback \
  /rosout

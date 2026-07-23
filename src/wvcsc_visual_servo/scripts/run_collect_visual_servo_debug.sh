#!/usr/bin/env bash
set -euo pipefail

# ── workspace setup ──────────────────────────────────────────────
set +u
source /opt/ros/humble/setup.bash

workspace_root="${WVCSC_WORKSPACE:-$HOME/WVCSC_S2Z_UTB_ARM}"
workspace_setup="$workspace_root/install/setup.bash"
if [[ ! -f "$workspace_setup" ]]; then
  echo "ERROR: Workspace setup not found: $workspace_setup" >&2
  echo "       Run colcon build first or set WVCSC_WORKSPACE." >&2
  exit 1
fi
source "$workspace_setup"
set -u

# ── output directory ─────────────────────────────────────────────
out_dir="$HOME/tmp/wvcsc/visual_servo_debug_$(date +%Y%m%d_%H%M)"
bag_dir="$out_dir/bag"
mkdir -p "$bag_dir"

echo "============================================================"
echo " WVCSC 视觉伺服调试数据收集"
echo " 输出目录: $out_dir"
echo "============================================================"
echo ""
echo " 请先启动仿真环境，确认所有节点就绪后按 Enter 开始录制。"
echo " 录制过程中按 Ctrl-C 停止。"
echo ""
read -rp " 按 Enter 开始录制... "

# ── record rosbag ────────────────────────────────────────────────
echo "[recording] 开始录制 rosbag ..."
set +e
ros2 bag record --include-hidden-topics -o "$bag_dir/servo_debug" \
  /clock \
  /vision/visual_servo_debug \
  /vision/target \
  /vision/tree_detections \
  /vision/diseased_target_detections \
  /vision/debug_image \
  /vision/perception_debug \
  /vision/selected_target_id \
  /vision/inference_mode \
  /servo_node/delta_twist_cmds \
  /servo_node/status \
  /servo_node/collision_velocity_scale \
  /joint_states \
  /arm_controller/joint_trajectory \
  /arm/observation_debug \
  /mission/status \
  /vision/align_target/_action/status \
  /vision/align_target/_action/feedback \
  /arm/execute_spray/_action/status \
  /arm/execute_spray/_action/feedback \
  /spray/execute/_action/status \
  /spray/execute/_action/feedback \
  /camera/color/camera_info \
  /tf \
  /tf_static \
  /rosout

# ros2 bag record exits with 130 on SIGINT — treat as normal
rec_exit=$?
set -e
if [[ $rec_exit -ne 0 ]] && [[ $rec_exit -ne 130 ]] && [[ $rec_exit -ne 2 ]]; then
  echo "ERROR: ros2 bag record exited with code $rec_exit" >&2
  exit $rec_exit
fi

echo ""
echo "[done] rosbag 录制完成"

# ── find the actual bag directory (ros2 bag adds a timestamp suffix) ──
actual_bag=$(find "$bag_dir" -maxdepth 2 -name '*.db3' -printf '%h\n' 2>/dev/null | head -1)
if [[ -z "$actual_bag" ]]; then
  echo "WARNING: No .db3 bag file found under $bag_dir" >&2
  actual_bag="$bag_dir/servo_debug"
fi
echo "[bag]   $actual_bag"

# ── bag info ─────────────────────────────────────────────────────
echo "[info]  写入 $out_dir/topics.txt ..."
ros2 bag info "$actual_bag" > "$out_dir/topics.txt" 2>&1 || true

# ── extract visual_servo_debug JSONL ─────────────────────────────
echo "[extract] 提取 /vision/visual_servo_debug → visual_servo_debug.jsonl ..."
extract_script="$(dirname "$0")/_extract_servo_debug.py"
if [[ -x "$extract_script" ]]; then
  python3 "$extract_script" \
    --bag "$actual_bag" \
    --topic /vision/visual_servo_debug \
    --out "$out_dir/visual_servo_debug.jsonl" \
    --summary "$out_dir/summary.txt" \
    2>&1 | sed 's/^/[extract] /'
else
  echo "WARNING: 提取脚本不可用: $extract_script" >&2
  echo "         visual_servo_debug.jsonl 将不会生成" >&2
fi

# ── final summary ────────────────────────────────────────────────
echo ""
echo "============================================================"
echo " 数据收集完成"
echo " 输出目录: $out_dir"
echo "============================================================"
ls -lh "$out_dir"/
echo ""
if [[ -f "$out_dir/summary.txt" ]]; then
  echo "── 事件摘要 ──"
  cat "$out_dir/summary.txt"
fi

#!/usr/bin/env bash
set -eo pipefail

# 这个脚本仍然只负责启动已经验证过的真实单臂后端。默认情况下它不会
# 发布运动、伺服或继电器命令；只有显式指定 auto_execute:=true 时，才
# 作为一个 ROS 2 Action 客户端发送一个 ExecuteSpray Goal。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"

auto_execute=false
auto_observation_mode=''
auto_side=''
auto_tree_distance_m=''
auto_working_range_m=''
auto_spray_duration_sec=''
auto_mission_id=''
auto_tree_id=''
observation_mode_launch_arg=''
launch_args=()

# ROS 2 launch 参数使用 key:=value。脚本私有的 auto_* 参数必须在这里
# 截留，不能传给 launch；其余参数原样透传，方便同事覆盖串口、标定和
# YOLO Python 解释器等现场配置。
for argument in "$@"; do
  case "$argument" in
    auto_execute:=*) auto_execute="${argument#auto_execute:=}" ;;
    observation_mode:=*)
      auto_observation_mode="${argument#observation_mode:=}"
      observation_mode_launch_arg="$argument"
      ;;
    auto_side:=*) auto_side="${argument#auto_side:=}" ;;
    auto_tree_distance_m:=*) auto_tree_distance_m="${argument#auto_tree_distance_m:=}" ;;
    auto_working_range_m:=*) auto_working_range_m="${argument#auto_working_range_m:=}" ;;
    auto_spray_duration_sec:=*) auto_spray_duration_sec="${argument#auto_spray_duration_sec:=}" ;;
    auto_mission_id:=*) auto_mission_id="${argument#auto_mission_id:=}" ;;
    auto_tree_id:=*) auto_tree_id="${argument#auto_tree_id:=}" ;;
    *) launch_args+=("$argument") ;;
  esac
done

die() {
  echo "[run_real_arm_spray_server] ERROR: $*" >&2
  exit 2
}

is_number() {
  [[ "$1" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]]
}

in_range() {
  awk -v value="$1" -v lower="$2" -v upper="$3" \
    'BEGIN { exit !(value >= lower && value <= upper) }'
}

require_value() {
  [[ -n "$2" ]] || die "$1 must be provided when auto_execute:=true"
}

validate_id() {
  [[ "$2" =~ ^[A-Za-z0-9_.-]+$ ]] ||
    die "$1 must contain only letters, numbers, '.', '_' or '-'"
}

case "${auto_execute,,}" in
  false|'') auto_execute=false ;;
  true) auto_execute=true ;;
  *) die "auto_execute must be true or false" ;;
esac

if [[ "$auto_execute" == false ]]; then
  # 无 auto_execute 时保持原行为：只启动后端，并把 launch 参数全部保留。
  if [[ -n "$observation_mode_launch_arg" ]]; then
    launch_args+=("$observation_mode_launch_arg")
  fi
  exec ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
    use_qt_gui:=false "${launch_args[@]}"
fi

# ROS 2 Action Goal 没有服务式的“调用后立即返回”语义。这里必须显式
# 提供每一个 Goal 字段，避免脚本悄悄使用与 Qt/完整任务不同的默认值。
require_value observation_mode "$auto_observation_mode"
require_value auto_side "$auto_side"
require_value auto_tree_distance_m "$auto_tree_distance_m"
require_value auto_working_range_m "$auto_working_range_m"
require_value auto_spray_duration_sec "$auto_spray_duration_sec"
require_value auto_mission_id "$auto_mission_id"
require_value auto_tree_id "$auto_tree_id"

auto_observation_mode="${auto_observation_mode,,}"
auto_side="${auto_side,,}"
[[ "$auto_observation_mode" == ik || "$auto_observation_mode" == joint_presets ]] ||
  die "observation_mode must be ik or joint_presets"
[[ "$auto_side" == left || "$auto_side" == right ]] ||
  die "auto_side must be left or right"

for numeric_name in auto_tree_distance_m auto_working_range_m auto_spray_duration_sec; do
  numeric_value="${!numeric_name}"
  is_number "$numeric_value" || die "$numeric_name must be a finite number"
done
in_range "$auto_tree_distance_m" 0.80 1.50 ||
  die "auto_tree_distance_m must be within 0.80-1.50 m"
in_range "$auto_spray_duration_sec" 0.20 10.00 ||
  die "auto_spray_duration_sec must be within 0.20-10.00 s"
# 0.0 preserves the normal dynamic tree/camera geometry calculation. A
# positive value is the standalone-test manual nozzle aim-plane override.
if ! awk -v value="$auto_working_range_m" \
    'BEGIN { exit (value == 0.0 || (value >= 0.20 && value <= 2.00)) ? 0 : 1 }'; then
  die "auto_working_range_m must be 0 or within 0.20-2.00 m"
fi
validate_id auto_mission_id "$auto_mission_id"
validate_id auto_tree_id "$auto_tree_id"

tree_y=''
if [[ "$auto_side" == left ]]; then
  tree_y="$auto_tree_distance_m"
else
  tree_y="-$(awk -v value="$auto_tree_distance_m" 'BEGIN { printf "%.6f", value }')"
fi

# 启动真实后端；SprayTask 是唯一的动作编排者，脚本不直接向
# /servo_node/delta_twist_cmds、/relay/set 或 /motion_control/command 写消息。
ros2 launch wvcsc_bringup real_arm_spray_test.launch.py \
  use_qt_gui:=false "${launch_args[@]}" &
backend_pid=$!

cleanup() {
  if kill -0 "$backend_pid" 2>/dev/null; then
    kill "$backend_pid" 2>/dev/null || true
    wait "$backend_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# 等待 Action Server，而不是用固定 sleep 猜测启动时序；后续 Goal 由
# wvcsc_spray_task 依次完成观测、检测、视觉伺服、喷嘴喷洒、复检和 HOME。
deadline=$((SECONDS + 120))
until ros2 action info /arm/execute_spray 2>/dev/null |
    grep -Eq 'Action servers:[[:space:]]*[1-9]'; do
  kill -0 "$backend_pid" 2>/dev/null || die 'real arm backend exited before Action Server became ready'
  (( SECONDS < deadline )) || die 'timed out waiting for /arm/execute_spray Action Server'
  sleep 1
done

# ros2 action send_goal 是真正的 Action 客户端调用。PointStamped 的 frame_id
# 固定为 alicia_base_link；左右侧只改变 Y 符号，x/z 在该单臂测试约定中为 0。
goal="{mission_id: \"${auto_mission_id}\", tree_id: \"${auto_tree_id}\", spray_duration: ${auto_spray_duration_sec}, tree_hint: {header: {frame_id: alicia_base_link}, point: {x: 0.0, y: ${tree_y}, z: 0.0}}, observation_mode: ${auto_observation_mode}, working_range_m: ${auto_working_range_m}}"
echo "[run_real_arm_spray_server] sending one ExecuteSpray Goal: mode=${auto_observation_mode} side=${auto_side} distance=${auto_tree_distance_m}m range=${auto_working_range_m}m duration=${auto_spray_duration_sec}s"
ros2 action send_goal /arm/execute_spray \
  wvcsc_interfaces/action/ExecuteSpray "$goal" -f

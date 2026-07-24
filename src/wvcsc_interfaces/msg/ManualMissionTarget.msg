string target_id
geometry_msgs/Pose docking_pose
float32 spray_duration
# ``POINT_INSPECT`` deliberately has value zero.  Existing callers created
# this message before route point kinds existed, so their default field value
# must retain the historical "navigate then arm spray" behavior.
uint8 POINT_INSPECT=0
uint8 POINT_TRANSIT=1
uint8 POINT_FINISH=2
uint8 point_type
# Enable channel-1 wide spraying only after Nav2 accepts this point and the
# vehicle is observed moving.  It is an incoming-segment property.
bool wide_spray_on_approach
# Optional stationary wait after the point is reached and the vehicle stops.
float32 dwell_time_sec
uint8 WORK_SIDE_UNSPECIFIED=0
uint8 WORK_SIDE_LEFT=1
uint8 WORK_SIDE_RIGHT=2
uint8 work_side
# Per-inspect-point override.  Zero falls back to the legacy spray_duration.
float32 arm_spray_duration_sec
# The measured-site loader uses confidence=1.0 and manual://measured;
# the simulation mock loader preserves its configured values.
float32 confidence
string evidence_uri
# Optional explicit tree-root position in the LoadManualMission header frame.
geometry_msgs/Point tree_hint
bool use_explicit_tree_hint
# Explicit signed tree position relative to alicia_base_link.  +X is forward
# and +Y is left.  This source is used with an explicitly recorded docking pose.
float32 tree_x_m
float32 tree_y_m
float32 tree_base_z_m
bool use_tree_offset_from_arm_base
# When true, MissionManager derives docking_pose from tree_hint and its single
# road/docking configuration.  RViz and measured-site clients leave this false.
bool compute_docking_pose

string target_id
geometry_msgs/Pose docking_pose
float32 spray_duration
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

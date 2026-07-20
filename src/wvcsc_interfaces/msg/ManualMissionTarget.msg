string target_id
geometry_msgs/Pose docking_pose
string spray_side
float32 spray_duration
# Optional explicit tree-root position in the LoadManualMission header frame.
# Existing clients keep the historical standoff-based inference by leaving
# use_explicit_tree_hint false.
geometry_msgs/Point tree_hint
bool use_explicit_tree_hint

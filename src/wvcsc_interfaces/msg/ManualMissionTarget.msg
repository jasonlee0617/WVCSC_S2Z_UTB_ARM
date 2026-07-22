string target_id
geometry_msgs/Pose docking_pose
string spray_side
float32 spray_duration
# The measured-site loader uses confidence=1.0 and manual://measured;
# the simulation mock loader preserves its configured values.
float32 confidence
string evidence_uri
# Optional explicit tree-root position in the LoadManualMission header frame.
geometry_msgs/Point tree_hint
bool use_explicit_tree_hint
# When true, MissionManager derives docking_pose from tree_hint and its single
# road/docking configuration.  RViz and measured-site clients leave this false.
bool compute_docking_pose

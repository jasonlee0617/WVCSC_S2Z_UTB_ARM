# 成功加载后，由 MissionManager 在 /mission/plan 话题上发布的只读任务摘要。
# 仿真和数据采集节点通过它进行可视化和检查。
# Qt 不将此用作完整任务的重载或编辑格式；LoadManualMission 依然是权威的输入契约。
std_msgs/Header header                 # 任务坐标系和发布时间戳。
string mission_id                      # 已加载的任务标识符。
wvcsc_interfaces/MissionTargetPlan[] targets  # 按执行顺序排列的任务目标摘要。
bool return_home_after_finish          # 请求的路径级 HOME 行为。
geometry_msgs/Pose home_pose           # header.frame_id 坐标系下的车辆 HOME 位姿。
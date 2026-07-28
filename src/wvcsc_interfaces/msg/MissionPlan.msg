# MissionManager 在成功加载任务后向 /mission/plan 发布的只读路线摘要。
# 仿真、数据采集和 Qt 可据此展示已加载路线；它不能用于编辑或重载任务，
# LoadManualMission 仍是唯一权威输入契约。
std_msgs/Header header                 # 路线公共坐标系和发布时间戳。
string mission_id                      # 已加载路线的任务标识符。
wvcsc_interfaces/MissionPointPlan[] points  # 按执行顺序排列的路线 Point 摘要。
bool return_home_after_mission         # 最后一个 Point 完成后是否发送 Nav2 HOME 目标。
geometry_msgs/Pose home_pose           # header.frame_id 下的车辆 HOME 位姿。

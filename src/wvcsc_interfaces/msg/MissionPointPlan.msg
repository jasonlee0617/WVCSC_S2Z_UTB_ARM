# MissionPlan 中每个路线 Point 的只读紧凑摘要。MissionManager 在成功加载后发布；
# 仿真、Qt 状态展示和数据采集节点可消费它，但不得将它当作 LoadManualMission 的回灌格式。
# 它保留执行分类和几何摘要，刻意省略可编辑的广域喷洒标志、停留时间和作业侧等输入细节。
string point_id                        # 路线 Point 标识符，不是病态视觉 target_id。
uint8 POINT_INSPECT=0                  # 到站停稳后需要触发 ExecuteSpray 的检查 Point。
uint8 POINT_TRANSIT=1                  # 不触发机械臂喷洒的通行 Point。
uint8 point_type                       # 上述路线分类，供只读消费者区分 Nav2 后续动作。
geometry_msgs/Point tree_hint          # MissionPlan.header.frame_id 下的植物根部几何提示。
float32 tree_x_m                       # 记录时相对 alicia_base_link 的前向偏移（米，+X 向前）。
float32 tree_y_m                       # 记录时相对 alicia_base_link 的侧向偏移（米，+Y 向左）。
float32 spray_duration                 # 该 Point 生效的每个病态目标喷嘴时长（秒）。
geometry_msgs/Pose docking_pose         # MissionPlan.header.frame_id 下的 Nav2 车辆基座目标位姿。

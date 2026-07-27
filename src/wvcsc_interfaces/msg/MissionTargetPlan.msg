# 只读 MissionPlan 发布消息中的每个目标的紧凑条目。
# 它提供了足够的几何信息供仿真和数据消费节点使用，但省略了可编辑字段
# （如点类型、广域喷洒标志、停留时间）；请勿将其作为完整的 LoadManualMission
# 目标进行重放。
string target_id                       # 路点标识符。
geometry_msgs/Point tree_hint          # MissionPlan.header.frame_id 坐标系下的树位置。
float32 tree_x_m                       # 相对于 alicia_base_link 的带符号前向偏移量（+X 向前）。
float32 tree_y_m                       # 相对于 alicia_base_link 的带符号侧向偏移量（+Y 向左）。
float32 spray_duration                 # 每个目标生效的请求喷洒时长（秒）。
geometry_msgs/Pose docking_pose         # 在计划头（plan header）坐标系下的 Nav2 车辆底盘目标位姿。
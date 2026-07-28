# Qt/RViz 手动记录的一条路线 Point，由 LoadManualMission 送入 MissionManager。
# Qt 创建并编辑它；MissionManager 校验、排序并拥有运行时调度权；Nav2、广域喷洒和
# ExecuteSpray 只消费其语义。它是可编辑输入，不是只读 MissionPlan 摘要。
# point_id 只标识路线点，不能与感知 Target2D.target_id（病态目标追踪 ID）混用。
string point_id                       # Qt 为该路线 Point 生成的稳定标识符。
geometry_msgs/Pose docking_pose        # 在 LoadManualMission.header.frame_id 下的 Nav2 车辆基座目标位姿。
float32 spray_duration                 # 每个病态目标的遗留喷嘴时长回退值（秒）。
# 仅两类路线 Point：最后一个数组元素自然结束任务，不再使用专门的 FINISH 类型。
# INSPECT 保持为 0，兼容未显式设置 point_type 的旧消息初始化语义。
uint8 POINT_INSPECT=0                  # Nav2 到达并确认车辆停稳后，MissionManager 发送 ExecuteSpray。
uint8 POINT_TRANSIT=1                  # 只导航/可选停留；绝不发送 ExecuteSpray。
uint8 point_type                       # 上述 Point 类型常量之一。
# 进入此 Point 的导航段属性。仅在 Nav2 Goal 被接受且里程计确认车辆开始运动后，
# MissionManager 才会请求打开广域喷洒继电器通道 1；它与 point_type 相互独立。
bool wide_spray_on_approach
float32 dwell_time_sec                 # 到站停稳后的可选静态等待时长（秒）。
uint8 WORK_SIDE_UNSPECIFIED=0          # 未选择作业侧。
uint8 WORK_SIDE_LEFT=1                 # 按 alicia_base_link 约定的 +Y 侧。
uint8 WORK_SIDE_RIGHT=2                # 按 alicia_base_link 约定的 -Y 侧。
# Qt/记录路线的作业侧元数据。它不作为 ExecuteSpray 的 ID 传递；机械臂几何使用 tree_* 提示。
uint8 work_side
# 每个 INSPECT Point 的喷嘴时长覆盖（秒）；0 时 MissionManager 回退到 spray_duration。
float32 arm_spray_duration_sec
# 相对于 alicia_base_link 的带符号植物根部几何提示：+X 向前、+Y 向左，Z 为根部基准高度。
# MissionManager 将这些记录值与导航位姿结合，形成 ExecuteSpray.tree_hint。
float32 tree_x_m
float32 tree_y_m
float32 tree_base_z_m

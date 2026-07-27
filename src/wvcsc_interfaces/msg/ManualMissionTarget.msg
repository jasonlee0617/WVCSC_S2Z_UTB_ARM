# 由 Qt/RViz 记录的一个路点，通过 LoadManualMission 服务携带。
# Qt 创建它，MissionManager 进行校验并拥有所有权，
# 下游的 Nav2/ExecuteSpray 步骤将消耗其语义字段。
# 这是可编辑的任务输入消息，而非只读的 MissionPlan 摘要。
string target_id                      # Qt 生成的稳定路点标识符。
geometry_msgs/Pose docking_pose        # 在任务头坐标系下的 Nav2 车辆底盘位姿。
float32 spray_duration                 # 遗留的每目标喷嘴时长回退值（秒）。
# POINT_INSPECT 刻意保持为零：在点类型引入前构建的调用方依赖于默认初始化的消息，
# 其含义为“导航至目标点，然后执行机械臂喷洒”。
uint8 POINT_INSPECT=0                  # 到达、停稳，并可能发送 ExecuteSpray。
uint8 POINT_TRANSIT=1                  # 纯导航点；不发送机械臂 Goal。
uint8 POINT_FINISH=2                   # 终端路点；强制关闭广域喷洒。
uint8 point_type                       # 上述点类型常量之一。
# 传入的导航段属性。仅当 Nav2 接受此点且新的里程计确认车辆开始运动后，
# 才会请求开启广域喷洒通道 1。
bool wide_spray_on_approach
float32 dwell_time_sec                 # 到达停车后的可选静态等待时间（秒）。
uint8 WORK_SIDE_UNSPECIFIED=0          # 未选择作业侧。
uint8 WORK_SIDE_LEFT=1                 # 按 alicia_base_link 约定，+Y 侧。
uint8 WORK_SIDE_RIGHT=2                # 按 alicia_base_link 约定，-Y 侧。
uint8 work_side                        # 给机械臂任务的观察侧提示。
# 每检查点的喷洒时长覆盖值；零值则回退到使用 spray_duration。
float32 arm_spray_duration_sec
# 相对于 alicia_base_link 的带符号树根位置：+X 向前，+Y 向左，
# Z 为树基高度。检查点使用此 Qt/RViz 记录的数据。
float32 tree_x_m
float32 tree_y_m
float32 tree_base_z_m
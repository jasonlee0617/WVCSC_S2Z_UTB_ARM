# MissionManager 的实时状态，通过 /mission/status 话题发布，供 Qt 界面和监控使用。
# 报告全局路径状态、当前目标以及子 Action 的活跃状态；
# 它本身不直接发送命令给 Nav2、继电器硬件或机械臂。
uint8 IDLE=0                           # 未加载/未运行任务。
uint8 WAITING_FOR_TASKS=1              # 等待有效的任务列表加载。
uint8 READY=2                          # 任务已加载完毕，可以启动。
uint8 NAVIGATING=3                     # 正在向某个路径点执行 Nav2 导航。
uint8 VERIFYING_STOP=4                 # 到达目标点，正在确认车辆是否停稳。
uint8 ARM_SPRAYING=5                   # 正在执行检查点 (INSPECT point) 的 ExecuteSpray 动作。
uint8 TARGET_COMPLETED=6               # 兼容性过渡状态。
uint8 PAUSED=7                         # 路径已暂停 / 等待恢复。
uint8 MISSION_COMPLETED=8              # 路径到达终点完成。
uint8 CANCELED=9                       # 路径取消的终止状态。
uint8 FAILED=10                        # 必需的安全/操作失败状态。
uint8 RETURNING_HOME=11                # Nav2 正在返回 HOME 位置。
uint8 DWELLING=12                      # 在某个点执行可选的静态等待。

std_msgs/Header header                 # 地图坐标系下的状态发布时间戳。
string mission_id                      # 当前加载的任务 ID，空闲时为空。
uint8 state                            # 上述全局状态常量之一。
string state_text                      # 供 Qt/日志显示的人类可读状态详情。
string current_tree_id                 # 当前路径点 ID；不一定是树。
uint32 current_index                   # 当前目标索引（从 0 开始）。
uint32 total_targets                   # 加载的路径目标总数。
uint32 completed_targets               # 已完全处理完成的路径目标数。
uint32 skipped_targets                 # 因可恢复故障而被跳过的路径目标数。
wvcsc_interfaces/MissionTargetStatus[] target_statuses  # 每个目标的执行结果视图。
string last_error                      # 最新的错误/详情，无可用信息时为空。
bool nav_goal_active                   # 子 Nav2 Action 是否处于激活状态。
bool arm_goal_active                   # 子 ExecuteSpray Action 是否处于激活状态。
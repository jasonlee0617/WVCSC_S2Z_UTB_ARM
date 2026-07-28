# MissionManager 的实时状态，通过 /mission/status 发布，供 Qt 和监控节点使用。
# 它报告路线状态、当前 Point 和子 Action 活跃性；它本身不直接向 Nav2、继电器或机械臂发命令。
# 当前状态只包含路线 Point，不表示视觉病态目标队列的逐目标状态。
uint8 IDLE=0                           # 未加载或未运行任务。
uint8 WAITING_FOR_TASKS=1              # 等待加载有效路线 Point 列表。
uint8 READY=2                          # 路线已加载，可接收 /mission/start。
uint8 NAVIGATING=3                     # 正在向当前 Point 发送/执行 Nav2 导航。
uint8 VERIFYING_STOP=4                 # Nav2 到站后，仅通过里程计确认车辆停稳。
uint8 ARM_SPRAYING=5                   # 当前 INSPECT Point 正在执行 ExecuteSpray。
# 数值 6 为已删除的 TARGET_COMPLETED 兼容空档，不能复用。
uint8 PAUSED=7                         # 路线暂停或等待恢复。
uint8 MISSION_COMPLETED=8              # 最后一个 Point 已完成，且无需或已完成 HOME。
uint8 CANCELED=9                       # 任务取消后的终止状态。
uint8 FAILED=10                        # 必需安全或操作失败导致的终止状态。
uint8 RETURNING_HOME=11                # Nav2 正在返回记录的 HOME 位姿。
uint8 DWELLING=12                      # 当前 Point 正在执行可选静态等待。

std_msgs/Header header                 # 当前状态的坐标系和发布时间戳。
string mission_id                      # 当前加载的任务 ID；空闲时为空。
uint8 state                            # 上述全局路线状态常量之一。
string state_text                      # 供 Qt/日志显示的人类可读状态详情。
string current_point_id                # 当前路线 Point ID；无当前 Point 时为空。
uint32 current_index                   # 当前 Point 的 0 基索引；无当前 Point 时不应单独作为有效索引解释。
uint32 total_points                    # 已加载路线 Point 总数。
uint32 completed_points                # 已正常完成的路线 Point 数。
uint32 skipped_points                  # 因可恢复故障被跳过的路线 Point 数。
wvcsc_interfaces/MissionPointStatus[] point_statuses  # 每个路线 Point 的执行状态视图。
string last_error                      # 最近错误/详情；无可用信息时为空。
bool nav_goal_active                   # 子 Nav2 Action 当前是否处于激活状态。
bool arm_goal_active                   # 子 ExecuteSpray Action 当前是否处于激活状态。

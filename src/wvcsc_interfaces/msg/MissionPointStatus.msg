# MissionStatus 中嵌套的一条路线 Point 运行状态。MissionManager 用它向 Qt 和监控节点
# 说明每个计划 Point 是待执行、当前执行、已完成、因可恢复问题跳过或失败。
# 这是路线调度状态，不是 YOLO 病态目标队列状态；病态目标由 Target2D.target_id 区分。
uint8 PENDING=0                        # 已加载，尚未轮到执行。
uint8 CURRENT=1                        # 当前正在导航、停稳、停留或执行机械臂动作的 Point。
uint8 COMPLETED=2                      # Point 的要求已正常完成。
uint8 SKIPPED=3                        # 因可恢复问题跳过，任务可继续后续 Point。
uint8 FAILED=4                         # Point 未完成或仅部分完成；细节见 message。

string point_id                        # 对应 ManualMissionPoint.point_id 的路线标识符。
uint8 state                            # 上述 Point 状态常量之一。
string state_text                      # 供 Qt 显示的简短状态文本。
string message                         # 详细诊断，例如相关 Action 的结果文本。

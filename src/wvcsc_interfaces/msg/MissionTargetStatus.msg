# MissionStatus 中嵌套的每个路径目标状态。
# MissionManager 通过它告知 Qt 界面，每个计划点的状态是待定、活跃、已完成还是已跳过。
# PARTIAL（部分喷洒）的 ExecuteSpray 结果有意映射到 FAILED 状态，并通过 state_text/message 字段提供说明，而不是增加新的网络常量。
uint8 PENDING=0                        # 已加载但尚未到达。
uint8 CURRENT=1                        # 当前正在执行的路径点。
uint8 COMPLETED=2                      # 路径点已正常完成。
uint8 SKIPPED=3                        # 因可恢复故障跳过了此点。
uint8 FAILED=4                         # 部分/未解决或失败的路径点结果。

string tree_id                         # 历史命名习惯；实际包含路点目标 ID。
uint8 state                            # 上述目标状态常量之一。
string state_text                      # 简短的可读结果状态。
string message                         # 详细结果，包括 Action 的结果文本。
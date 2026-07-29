# 中文说明：单个 ExecuteSpray Goal 的可变工作流状态。
# 该模块只保存目标账本、计数和阶段数据，不创建 ROS 通信，也不直接控制机械臂。
"""Per-goal mutable state for the spray workflow."""

from dataclasses import dataclass, field

from .target_ledger import spray_summary, target_accounting


@dataclass
class SpraySession:
    """State owned by one ``ExecuteSpray`` goal, never by the ROS node."""

    processed: list = field(default_factory=list)
    exhausted: list = field(default_factory=list)
    known_targets: list = field(default_factory=list)
    attempts: list = field(default_factory=list)
    pending_attempt: object = None
    sprayed: int = 0
    saw_disease: bool = False
    alignment_failures: int = 0
    recenter_attempts: int = 0
    recenter_failures: int = 0
    alignment_attempts: int = 0
    last_alignment_feedback_at: float = 0.0

    def accounting(self, same_target):
        return target_accounting(
            self.known_targets,
            self.processed,
            self.exhausted,
            same_target,
        )

    def result_summary(self, detected, accounted_sprayed, unresolved):
        return spray_summary(
            detected,
            accounted_sprayed,
            unresolved,
            self.alignment_failures,
            self.recenter_attempts,
            self.recenter_failures,
            self.alignment_attempts,
        )

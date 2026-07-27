"""Per-goal mutable state for the spray workflow."""

from dataclasses import dataclass, field

from .target_flow import spray_summary, target_accounting


@dataclass
class SpraySession:
    """State owned by one ``ExecuteSpray`` goal, never by the ROS node."""

    tree_id: str
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


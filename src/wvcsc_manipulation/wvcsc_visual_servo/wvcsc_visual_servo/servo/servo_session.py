"""Per-goal visual-servo decisions without ROS transport or Action side effects."""

from dataclasses import dataclass, field
from enum import Enum
import math

from .alignment_progress import AlignmentProgress
from .target_tracker import TargetState, TargetTracker


class ServoDecisionKind(str, Enum):
    HOLD = 'hold'
    COMMAND = 'command'
    SUCCESS = 'success'
    FAIL = 'fail'


class ServoFailureKind(str, Enum):
    INVALID_GOAL = 'invalid_goal'
    TIMEOUT = 'timeout'
    TARGET_STALE = 'target_stale'
    CANCELED = 'canceled'
    SERVO_SINGULARITY = 'servo_singularity'
    SERVO_SAFETY_STOP = 'servo_safety_stop'
    SERVO_ACTUATION_STALL = 'servo_actuation_stall'
    SERVO_DIRECTION_DIVERGENCE = 'servo_direction_divergence'


@dataclass(frozen=True)
class ServoDecision:
    kind: ServoDecisionKind
    latest: dict | None = None
    command: tuple = (0.0, 0.0)
    failure: ServoFailureKind | None = None
    message: str = ''
    stable_duration_sec: float = 0.0


@dataclass(frozen=True)
class DirectionGuardConfig:
    enabled: bool
    window_sec: float
    min_error_px: float
    max_growth_px: float
    min_confidence: float


@dataclass
class ServoSessionState:
    target: TargetState = field(default_factory=TargetState)
    started: float = 0.0
    last_control_monotonic: float = 0.0
    stop_failure: ServoFailureKind | None = None
    stop_message: str = ''
    alignment_hold_latched: bool = False
    direction_guard_baseline: tuple | None = None
    direction_guard_checked: bool = False


@dataclass(frozen=True)
class TerminalReport:
    snapshot: dict | None
    summary: str


class ServoSession:
    """State machine for one visual alignment goal.

    The ROS node serializes calls under its existing lock.  This class only
    owns control-domain state and therefore remains directly unit-testable.
    """

    def __init__(self, config, controller, actuation_monitor, direction_guard):
        self._config = config
        self._controller = controller
        self._monitor = actuation_monitor
        self._direction_guard = direction_guard
        self._progress = AlignmentProgress(
            config.fine_tolerance_px,
            config.stable_duration_sec,
            config.progress_window_sec,
            config.min_progress_px,
            config.control_resume_tolerance_px,
        )
        self._tracker = TargetTracker(config, self._progress)
        self._joint_positions = []
        self.state = ServoSessionState()

    @property
    def latest(self):
        return self.state.target.latest

    @property
    def stable_duration(self):
        return self._progress.stable_duration

    @property
    def last_command(self):
        return self._controller.last_command

    def twist_components(self, command):
        return self._controller.twist_components(command)

    def reset(self, started, control_started, joint_positions=()):
        self.state = ServoSessionState(
            started=float(started),
            last_control_monotonic=float(control_started),
        )
        self._joint_positions = list(joint_positions)
        self._controller.reset()
        self._progress.reset()
        self._monitor.reset(joint_positions)

    def observe_target(self, message, now, camera, desired_pixel):
        """Store one matching target and return whether immediate zero is needed."""
        if self._tracker.is_valid(message):
            self._tracker.update_valid(
                self.state.target, message, now, camera, desired_pixel)
            return False
        return self._tracker.update_invalid(self.state.target, message, now)

    def observe_joint_state(self, positions, active):
        self._joint_positions = list(positions)
        self._monitor.observe_joint_state(positions, active)

    def observe_servo_output(self, message, active, observed_at):
        self._monitor.observe_output(
            message, self._joint_positions, active, observed_at)

    def request_stop(self, failure, message):
        self.state.stop_failure = failure
        self.state.stop_message = str(message)

    def hold_zero(self):
        self._controller.hold_zero()
        self._monitor.end_motion()

    def step(self, now, monotonic_now, timeout, camera_ready, servo_status):
        """Return the next action without publishing or sleeping."""
        latest = self._copy_latest()
        if self.state.stop_failure is not None:
            return self._fail(self.state.stop_failure, self.state.stop_message, latest)

        if now - self.state.started >= timeout:
            failure = (
                ServoFailureKind.TARGET_STALE
                if latest is None or not latest.get('valid')
                else ServoFailureKind.TIMEOUT)
            return self._fail(
                failure,
                ('target unavailable/stale'
                 if failure == ServoFailureKind.TARGET_STALE
                 else 'visual alignment timed out'),
                latest)

        if not self._target_is_fresh(now, latest):
            unavailable_duration = max(
                0.0, now - self.state.target.target_unavailable_since)
            if unavailable_duration >= self._config.stale_timeout_sec:
                return self._fail(
                    ServoFailureKind.TARGET_STALE,
                    'target continuously unavailable for '
                    f'{unavailable_duration:.2f}s',
                    latest)
            return ServoDecision(ServoDecisionKind.HOLD, latest=latest)
        if not camera_ready:
            return ServoDecision(ServoDecisionKind.HOLD, latest=latest)

        direction_guard = self._direction_guard_result(now, latest)
        if direction_guard:
            return self._fail(
                ServoFailureKind.SERVO_DIRECTION_DIVERGENCE,
                'image-axis direction guard stopped Servo: ' + direction_guard,
                latest)

        if self._progress.aligned:
            return ServoDecision(
                ServoDecisionKind.SUCCESS,
                latest=latest,
                stable_duration_sec=self._progress.stable_duration,
            )
        if self._progress.stalled(now):
            failure = (
                ServoFailureKind.SERVO_SINGULARITY
                if servo_status == 6 else ServoFailureKind.TIMEOUT)
            return self._fail(
                failure,
                ('visual alignment stalled while leaving singularity'
                 if failure == ServoFailureKind.SERVO_SINGULARITY
                 else 'visual alignment stalled'),
                latest)

        if self._alignment_hold(latest):
            return ServoDecision(ServoDecisionKind.HOLD, latest=latest)

        elapsed = max(0.0, float(monotonic_now) - self.state.last_control_monotonic)
        self.state.last_control_monotonic = float(monotonic_now)
        command = self._controller.command(latest['error'], elapsed)
        if math.hypot(*command) > 1e-6:
            self._monitor.begin_motion(monotonic_now, self._joint_positions)
            stall_reason = self._monitor.stall_reason(monotonic_now)
            if stall_reason:
                return self._fail(
                    ServoFailureKind.SERVO_ACTUATION_STALL,
                    'servo actuation stalled: ' + stall_reason,
                    latest)
        return ServoDecision(
            ServoDecisionKind.COMMAND,
            latest=latest,
            command=command,
        )

    def terminal_snapshot(self, now, latest=None):
        return self._tracker.terminal_snapshot(self.state.target, now, latest)

    def terminal_report(self, failure, message, now, servo_status, camera_ready,
                        snapshot=None):
        snapshot = (
            self.terminal_snapshot(now, snapshot)
            if snapshot is None else dict(snapshot))
        diagnostics = self._monitor.diagnostics()
        error_u = 0.0 if snapshot is None else float(snapshot.get('error_u', 0.0))
        error_v = 0.0 if snapshot is None else float(snapshot.get('error_v', 0.0))
        age = -1.0 if snapshot is None else max(
            0.0, float(now) - float(snapshot.get('received', now)))
        unavailable = 0.0 if snapshot is None else float(
            snapshot.get('target_unavailable_sec', 0.0))
        stable_frames = 0 if snapshot is None else snapshot.get('stable_frames', 0)
        target_hold = False if snapshot is None else snapshot.get('hold', False)
        command = self._controller.last_command
        summary = (
            f'[VISUAL_SERVO] {failure.value} target_age={age:.2f}s '
            f'target_unavailable={unavailable:.2f}s '
            f'error_px=({error_u:.1f},{error_v:.1f}) '
            f'stable_frames={stable_frames} camera_ready={camera_ready} '
            f'target_hold={target_hold} '
            f'cmd_angular_rps=({command[0]:.3f},{command[1]:.3f}) '
            f'servo_outputs={diagnostics.output_count} '
            f'servo_output_rate_hz={diagnostics.output_rate_hz:.1f} '
            f'commanded_joint_delta={diagnostics.commanded_joint_delta_rad:.5f}rad '
            f'actual_joint_delta={diagnostics.actual_joint_delta_rad:.5f}rad '
            f'servo_status={servo_status} message={message}')
        return TerminalReport(snapshot, summary)

    def _copy_latest(self):
        return dict(self.state.target.latest) if self.state.target.latest is not None else None

    def _target_is_fresh(self, now, latest):
        if (latest is not None and latest.get('valid') and not latest.get('hold')
                and now - latest['received'] <= self._config.stale_timeout_sec):
            return True
        if self.state.target.target_unavailable_since is None:
            self.state.target.target_unavailable_since = (
                self.state.started if latest is None
                else float(latest.get('received', now)))
        return False

    def _alignment_hold(self, latest):
        error_norm = math.hypot(latest['error_u'], latest['error_v'])
        if error_norm <= self._config.fine_tolerance_px:
            self.state.alignment_hold_latched = True
            return True
        if (self.state.alignment_hold_latched and
                error_norm <= self._config.control_resume_tolerance_px):
            return False
        self.state.alignment_hold_latched = False
        return False

    def _direction_guard_result(self, now, latest):
        config = self._direction_guard
        if (not config.enabled or
                float(latest.get('confidence', 0.0)) < config.min_confidence):
            return None
        error = (float(latest['error_u']), float(latest['error_v']))
        if not all(math.isfinite(value) for value in error):
            return None
        state = self.state
        if state.direction_guard_checked:
            return None
        if state.direction_guard_baseline is None:
            if max(abs(value) for value in error) < config.min_error_px:
                return None
            state.direction_guard_baseline = (now, *error)
        baseline = state.direction_guard_baseline
        if now - baseline[0] < config.window_sec:
            return None
        state.direction_guard_checked = True
        _, baseline_u, baseline_v = baseline
        violations = []
        for axis, initial, current in (
            ('u', float(baseline_u), error[0]),
            ('v', float(baseline_v), error[1]),
        ):
            if (abs(initial) >= config.min_error_px and
                    abs(current) >= abs(initial) + config.max_growth_px):
                violations.append(
                    f'{axis}:abs_error={abs(initial):.1f}px→{abs(current):.1f}px')
        return '; '.join(violations)

    @staticmethod
    def _fail(failure, message, latest):
        return ServoDecision(
            ServoDecisionKind.FAIL,
            latest=latest,
            failure=failure,
            message=message,
        )

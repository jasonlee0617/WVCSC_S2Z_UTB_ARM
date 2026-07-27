"""Downstream MoveIt Servo output and joint-response monitoring."""

from dataclasses import dataclass


@dataclass
class ActuationState:
    initial_joint_positions: tuple = ()
    max_joint_delta_rad: float = 0.0
    output_count: int = 0
    output_first_monotonic: float | None = None
    output_last_monotonic: float | None = None
    max_commanded_joint_delta_rad: float = 0.0
    first_motion_command_monotonic: float | None = None
    motion_command_active: bool = False
    motion_epoch_joint_positions: tuple = ()
    motion_epoch_max_joint_delta_rad: float = 0.0


@dataclass(frozen=True)
class ActuationDiagnostics:
    output_count: int
    output_rate_hz: float
    commanded_joint_delta_rad: float
    actual_joint_delta_rad: float


class ActuationMonitor:
    """Observe the Twist -> JointTrajectory -> JointState execution chain."""

    def __init__(
        self,
        joint_names,
        response_timeout_sec,
        min_output_rate_hz,
        min_commanded_joint_delta_rad,
        min_actual_joint_delta_rad,
    ):
        self._joint_names = tuple(joint_names)
        self._response_timeout_sec = float(response_timeout_sec)
        self._min_output_rate_hz = float(min_output_rate_hz)
        self._min_commanded_joint_delta_rad = float(
            min_commanded_joint_delta_rad)
        self._min_actual_joint_delta_rad = float(min_actual_joint_delta_rad)
        self.state = ActuationState()

    def reset(self, initial_joint_positions=()):
        self.state = ActuationState(
            initial_joint_positions=tuple(initial_joint_positions))

    def observe_joint_state(self, positions, busy):
        positions = tuple(float(value) for value in positions)
        state = self.state
        initial = state.initial_joint_positions
        if busy and len(initial) == len(positions):
            state.max_joint_delta_rad = max(
                state.max_joint_delta_rad,
                max(abs(current - start)
                    for current, start in zip(positions, initial)),
            )
        motion_initial = state.motion_epoch_joint_positions
        if state.motion_command_active and len(motion_initial) == len(positions):
            state.motion_epoch_max_joint_delta_rad = max(
                state.motion_epoch_max_joint_delta_rad,
                max(abs(current - start)
                    for current, start in zip(positions, motion_initial)),
            )

    def begin_motion(self, now, joint_positions):
        state = self.state
        if state.motion_command_active:
            return
        state.motion_command_active = True
        state.first_motion_command_monotonic = float(now)
        state.motion_epoch_joint_positions = tuple(joint_positions)
        state.motion_epoch_max_joint_delta_rad = 0.0
        state.output_count = 0
        state.output_first_monotonic = None
        state.output_last_monotonic = None
        state.max_commanded_joint_delta_rad = 0.0

    def end_motion(self):
        self.state.motion_command_active = False

    def observe_output(self, message, joint_positions, busy, observed_at):
        if not message.points:
            return
        state = self.state
        if not busy or not state.motion_command_active:
            return
        state.output_count += 1
        if state.output_first_monotonic is None:
            state.output_first_monotonic = float(observed_at)
        state.output_last_monotonic = float(observed_at)

        point = message.points[0]
        if len(point.positions) != len(message.joint_names):
            return
        current = dict(zip(self._joint_names, joint_positions))
        commanded = dict(zip(message.joint_names, point.positions))
        if all(name in commanded and name in current
               for name in self._joint_names):
            state.max_commanded_joint_delta_rad = max(
                state.max_commanded_joint_delta_rad,
                max(
                    abs(float(commanded[name]) - float(current[name]))
                    for name in self._joint_names
                ),
            )

    def diagnostics(self):
        state = self.state
        rate = 0.0
        first = state.output_first_monotonic
        last = state.output_last_monotonic
        if first is not None and last is not None and last > first:
            if state.output_count > 1:
                rate = float(state.output_count - 1) / (last - first)
        return ActuationDiagnostics(
            output_count=state.output_count,
            output_rate_hz=rate,
            commanded_joint_delta_rad=state.max_commanded_joint_delta_rad,
            actual_joint_delta_rad=state.max_joint_delta_rad,
        )

    def stall_reason(self, now):
        state = self.state
        if (
            not state.motion_command_active
            or state.first_motion_command_monotonic is None
        ):
            return None
        elapsed = max(
            0.0, float(now) - state.first_motion_command_monotonic)
        if elapsed < self._response_timeout_sec:
            return None
        if state.output_last_monotonic is None:
            return (
                'no JointTrajectory received after '
                f'{elapsed:.2f}s of non-zero visual-servo command')
        output_age = max(
            0.0, float(now) - state.output_last_monotonic)
        if output_age > self._response_timeout_sec:
            return (
                'JointTrajectory output stopped for '
                f'{output_age:.2f}s after visual-servo command')
        first = state.output_first_monotonic
        last = state.output_last_monotonic
        if (
            first is not None
            and last > first
            and state.output_count > 1
        ):
            rate = float(state.output_count - 1) / (last - first)
            if rate < self._min_output_rate_hz:
                return (
                    f'JointTrajectory output rate {rate:.1f}Hz is below '
                    f'{self._min_output_rate_hz:.1f}Hz')
        actual_delta = (
            state.motion_epoch_max_joint_delta_rad
            if state.motion_epoch_joint_positions
            else state.max_joint_delta_rad
        )
        if (
            state.max_commanded_joint_delta_rad
            >= self._min_commanded_joint_delta_rad
            and actual_delta < self._min_actual_joint_delta_rad
        ):
            return (
                'joint state did not follow Servo trajectory '
                f'(commanded={state.max_commanded_joint_delta_rad:.5f}rad, '
                f'actual={actual_delta:.5f}rad)')
        return None

"""Pure state helpers for the WVCSC command-velocity safety gate."""

from dataclasses import dataclass
import math


RUNNING = 'RUNNING'
STOPPED_LOCKED = 'STOPPED_LOCKED'
RESETTING = 'RESETTING'
HOME_LOCKED = 'HOME_LOCKED'
RESET_FAILED = 'RESET_FAILED'


@dataclass(frozen=True)
class Freshness:
    """Latest monotonic receive times used by the real-time safety gate."""

    command: float | None = None
    odom: float | None = None
    scan: float | None = None
    imu: float | None = None


def is_fresh(received_at, now, timeout):
    """Return whether a required input was received within ``timeout`` seconds."""
    return (
        received_at is not None
        and math.isfinite(float(received_at))
        and 0.0 <= float(now) - float(received_at) <= float(timeout)
    )


def base_is_stopped(linear_x, angular_z, linear_limit, angular_limit):
    """Conservative 2-D stop predicate shared by runtime code and tests."""
    values = (linear_x, angular_z, linear_limit, angular_limit)
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return (
        abs(float(linear_x)) <= float(linear_limit)
        and abs(float(angular_z)) <= float(angular_limit)
    )


def velocity_allowed(*, autonomy_enabled, stop_latched, emergency_stop, freshness,
                     now, command_timeout, odom_timeout, scan_timeout, imu_timeout):
    """Evaluate all independent gates before forwarding a Nav2 command."""
    return (
        bool(autonomy_enabled)
        and not bool(stop_latched)
        and not bool(emergency_stop)
        and is_fresh(freshness.command, now, command_timeout)
        and is_fresh(freshness.odom, now, odom_timeout)
        and is_fresh(freshness.scan, now, scan_timeout)
        and is_fresh(freshness.imu, now, imu_timeout)
    )


def latch_can_clear(*, emergency_stop, recovery_active, arm_state):
    """Require a completed HOME recovery before a latched stop can clear."""
    return (
        not bool(emergency_stop)
        and not bool(recovery_active)
        and str(arm_state) == HOME_LOCKED
    )

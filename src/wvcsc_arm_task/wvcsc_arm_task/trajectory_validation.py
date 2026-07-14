"""Validation helpers for fail-closed trajectory execution."""


def duration_nanoseconds(duration):
    return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)


def valid_retimed_trajectory(trajectory):
    """Return true only for a non-empty trajectory with increasing timestamps."""
    if trajectory is None or not trajectory.joint_names or not trajectory.points:
        return False

    joint_count = len(trajectory.joint_names)
    previous_time = -1
    for point in trajectory.points:
        if len(point.positions) != joint_count:
            return False
        timestamp = duration_nanoseconds(point.time_from_start)
        if timestamp < 0 or timestamp <= previous_time:
            return False
        previous_time = timestamp
    return True


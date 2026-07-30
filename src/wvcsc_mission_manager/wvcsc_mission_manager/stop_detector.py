# stop_detector.py
# ============================================================================
# 车辆停稳判定（纯逻辑，不依赖 ROS2）
# ============================================================================


class StopDetector:
    """Require fresh odometry to remain below both speed limits before arm motion."""

    WAITING = 'waiting'
    STABLE = 'stable'
    STALE = 'stale'
    TIMEOUT = 'timeout'

    def __init__(
            self, linear_threshold=0.03, angular_threshold=0.03,
            stable_duration=1.0, stale_timeout=1.0, timeout=5.0):
        self.linear_threshold = float(linear_threshold)
        self.angular_threshold = float(angular_threshold)
        self._default_stable_duration = float(stable_duration)
        self.stable_duration = self._default_stable_duration
        self.stale_timeout = float(stale_timeout)
        self.timeout = float(timeout)
        self.active = False
        self.started_at = None
        self.last_update = None
        self.stable_since = None

    def start(self, now, stable_duration=None):
        """Start a stop check, optionally overriding this check's duration.

        The default keeps the detector's configured duration.  MissionManager
        uses a shorter per-check duration for transit points while retaining
        the longer arm-safety duration for inspection points.
        """
        self.active = True
        self.started_at = float(now)
        self.last_update = None
        self.stable_since = None
        if stable_duration is None:
            self.stable_duration = self._default_stable_duration
        else:
            stable_duration = float(stable_duration)
            if stable_duration <= 0.0:
                raise ValueError('stable_duration must be positive')
            self.stable_duration = stable_duration

    def update(self, now, linear_speed, angular_speed):
        if not self.active:
            return
        now = float(now)
        self.last_update = now
        stopped = (
            abs(linear_speed) <= self.linear_threshold
            and abs(angular_speed) <= self.angular_threshold
        )
        if stopped and self.stable_since is None:
            self.stable_since = now
        elif not stopped:
            self.stable_since = None

    def status(self, now):
        if not self.active:
            return self.WAITING
        now = float(now)
        if now - self.started_at >= self.timeout:
            return self.TIMEOUT
        freshness_origin = (
            self.last_update if self.last_update is not None else self.started_at)
        if now - freshness_origin >= self.stale_timeout:
            return self.STALE
        if (self.stable_since is not None and
                now - self.stable_since >= self.stable_duration):
            return self.STABLE
        return self.WAITING

    def stop(self):
        self.active = False
        self.started_at = None
        self.last_update = None
        self.stable_since = None

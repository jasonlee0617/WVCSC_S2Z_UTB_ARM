import math


class AlignmentProgress:
    """Track time-based convergence and meaningful error reduction."""

    def __init__(
            self, fine_tolerance_px, stable_duration_sec,
            progress_window_sec, min_progress_px):
        self.fine_tolerance_px = float(fine_tolerance_px)
        self.stable_duration_sec = float(stable_duration_sec)
        self.progress_window_sec = float(progress_window_sec)
        self.min_progress_px = float(min_progress_px)
        self.reset()

    def reset(self):
        self._stable_since = None
        self._stable_last = None
        self._progress_since = None
        self._progress_reference = None

    def reset_stable(self):
        self._stable_since = None
        self._stable_last = None

    def restart_progress(self, error_u_px, error_v_px, now):
        """Start a fresh watchdog window after target reacquisition."""
        self._progress_since = float(now)
        self._progress_reference = math.hypot(
            float(error_u_px), float(error_v_px))

    def update(self, error_u_px, error_v_px, now):
        now = float(now)
        norm = math.hypot(float(error_u_px), float(error_v_px))
        within_tolerance = (
            abs(float(error_u_px)) <= self.fine_tolerance_px
            and abs(float(error_v_px)) <= self.fine_tolerance_px)
        if within_tolerance:
            if self._stable_since is None:
                self._stable_since = now
            self._stable_last = now
        else:
            self.reset_stable()

        if self._progress_since is None:
            self._progress_since = now
            self._progress_reference = norm
        elif self._progress_reference - norm >= self.min_progress_px:
            self._progress_since = now
            self._progress_reference = norm
        return norm

    @property
    def stable_duration(self):
        if self._stable_since is None or self._stable_last is None:
            return 0.0
        return max(0.0, self._stable_last - self._stable_since)

    @property
    def aligned(self):
        return self.stable_duration >= self.stable_duration_sec

    def stalled(self, now):
        return (
            self._progress_since is not None
            and float(now) - self._progress_since >= self.progress_window_sec)

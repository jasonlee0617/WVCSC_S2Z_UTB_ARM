import math
import threading


class SprayInterlock:
    def __init__(self, min_duration=0.2, max_duration=10.0):
        self.min_duration = float(min_duration)
        self.max_duration = float(max_duration)
        self._lock = threading.Lock()
        self._active = False
        self._emergency_stopped = False

    @property
    def active(self):
        with self._lock:
            return self._active

    @property
    def emergency_stopped(self):
        with self._lock:
            return self._emergency_stopped

    def validate(self, mission_id, tree_id, duration, mode):
        if not str(mission_id).strip() or not str(tree_id).strip():
            return 'mission_id and tree_id are required'
        if not math.isfinite(duration):
            return 'duration must be finite'
        if not self.min_duration <= duration <= self.max_duration:
            return 'duration out of range'
        if mode != 'continuous':
            return 'mode must be continuous'
        return ''

    def claim(self):
        with self._lock:
            if self._active or self._emergency_stopped:
                return False
            self._active = True
            return True

    def release(self):
        with self._lock:
            self._active = False

    def set_emergency_stop(self, active):
        with self._lock:
            self._emergency_stopped = bool(active)

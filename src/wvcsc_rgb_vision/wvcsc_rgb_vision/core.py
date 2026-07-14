import math
import threading


class AlignmentTracker:
    WAITING = 'waiting'
    STALE = 'stale'
    ALIGNED = 'aligned'

    def __init__(
            self, tolerance_u=20.0, tolerance_v=20.0,
            min_confidence=0.7, stable_frames=5, stale_timeout=0.5):
        self.tolerance_u = float(tolerance_u)
        self.tolerance_v = float(tolerance_v)
        self.min_confidence = float(min_confidence)
        self.required_stable_frames = int(stable_frames)
        self.stale_timeout = float(stale_timeout)
        self._lock = threading.Lock()
        self._latest = None
        self._stable_key = None
        self._stable_frames = 0

    def reset(self):
        """Require a fresh run of stable frames for the next alignment goal."""
        with self._lock:
            self._latest = None
            self._stable_key = None
            self._stable_frames = 0

    def update(
            self, stamp, mission_id, tree_id, valid, confidence, center_u, center_v,
            image_width, image_height):
        values = (stamp, confidence, center_u, center_v)
        if not all(math.isfinite(value) for value in values):
            return False
        if (image_width <= 0 or image_height <= 0 or
                not str(mission_id).strip() or not str(tree_id).strip()):
            return False
        error_u = float(center_u) - float(image_width) / 2.0
        error_v = float(center_v) - float(image_height) / 2.0
        centered = (
            bool(valid)
            and confidence >= self.min_confidence
            and abs(error_u) <= self.tolerance_u
            and abs(error_v) <= self.tolerance_v
        )
        with self._lock:
            key = (mission_id, tree_id)
            if centered and key == self._stable_key:
                self._stable_frames += 1
            elif centered:
                self._stable_key = key
                self._stable_frames = 1
            else:
                self._stable_key = None
                self._stable_frames = 0
            self._latest = {
                'stamp': float(stamp),
                'mission_id': mission_id,
                'tree_id': tree_id,
                'error_u': error_u,
                'error_v': error_v,
                'stable_frames': self._stable_frames,
            }
        return True

    def status(self, now, mission_id, tree_id, since=0.0):
        with self._lock:
            latest = dict(self._latest) if self._latest is not None else None
        if latest is None or latest['stamp'] < since:
            return self.STALE, 0.0, 0.0, 0
        if now - latest['stamp'] > self.stale_timeout:
            return (
                self.STALE, latest['error_u'], latest['error_v'],
                latest['stable_frames'])
        if (latest['mission_id'] != mission_id or
                latest['tree_id'] != tree_id):
            return (
                self.WAITING, latest['error_u'], latest['error_v'], 0)
        if latest['stable_frames'] >= self.required_stable_frames:
            return (
                self.ALIGNED, latest['error_u'], latest['error_v'],
                latest['stable_frames'])
        return (
            self.WAITING, latest['error_u'], latest['error_v'],
            latest['stable_frames'])

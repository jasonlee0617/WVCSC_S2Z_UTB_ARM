"""Target-message validation and per-goal image tracking."""

from dataclasses import dataclass
import math


@dataclass
class TargetState:
    """Target-only state for one AlignTarget goal."""

    latest: dict | None = None
    last_valid_target: dict | None = None
    target_unavailable_since: float | None = None
    stable_frames: int = 0


class TargetTracker:
    """Translate matching Target2D messages into control-loop snapshots."""

    def __init__(self, config, progress):
        self._config = config
        self._progress = progress

    def is_valid(self, message):
        return (
            bool(message.valid)
            and math.isfinite(message.confidence)
            and message.confidence >= self._config.min_confidence
            and message.image_width > 0
            and message.image_height > 0
            and math.isfinite(message.center_u)
            and math.isfinite(message.center_v)
            and 0.0 <= message.center_u < message.image_width
            and 0.0 <= message.center_v < message.image_height
        )

    def update_invalid(self, state, message, now):
        """Update loss/hold state and return whether zero speed is required."""
        if state.target_unavailable_since is None:
            state.target_unavailable_since = now
        latest = state.latest
        if (
            latest is not None
            and latest.get('valid')
            and now - latest['received'] <= self._config.invalid_target_hold_sec
        ):
            state.stable_frames = 0
            self._progress.reset_stable()
            state.latest = {
                **latest,
                'hold': True,
                'stable_frames': 0,
            }
            return True
        state.stable_frames = 0
        self._progress.reset_stable()
        state.latest = {
            'valid': False,
            'received': now,
            'confidence': float(message.confidence),
            'hold': False,
        }
        return False

    def update_valid(self, state, message, now, camera, desired_pixel):
        reacquired = state.target_unavailable_since is not None
        state.target_unavailable_since = None

        desired_u, desired_v = desired_pixel(
            message.image_width, message.image_height)
        error_u = float(message.center_u) - desired_u
        error_v = float(message.center_v) - desired_v
        if camera is None:
            raise RuntimeError('CameraInfo is required before target control')
        fx, fy = camera[:2]
        error = (error_u / fx, error_v / fy)

        if math.hypot(error_u, error_v) <= self._config.fine_tolerance_px:
            state.stable_frames += 1
        else:
            state.stable_frames = 0
        if reacquired:
            self._progress.restart_progress(error_u, error_v, now)
        self._progress.update(error_u, error_v, now)

        state.latest = {
            'valid': True,
            'received': now,
            'error': error,
            'error_u': error_u,
            'error_v': error_v,
            'confidence': float(message.confidence),
            'stable_frames': state.stable_frames,
            'hold': False,
        }
        state.last_valid_target = dict(state.latest)

    @staticmethod
    def terminal_snapshot(state, now, latest=None):
        """Freeze the final useful target before ROS braking can age it."""
        current = (
            dict(state.latest) if latest is None and state.latest is not None
            else (dict(latest) if latest is not None else None))
        last_valid = (
            dict(state.last_valid_target)
            if state.last_valid_target is not None else None)
        snapshot = current if current is not None and current.get('valid') else last_valid
        if snapshot is None:
            snapshot = current
        if snapshot is None:
            return None
        snapshot = dict(snapshot)
        snapshot['terminal_target_valid'] = bool(
            current is not None and current.get('valid')
            and not current.get('hold'))
        snapshot['target_unavailable_sec'] = (
            0.0 if state.target_unavailable_since is None
            else max(0.0, float(now) - float(state.target_unavailable_since)))
        return snapshot

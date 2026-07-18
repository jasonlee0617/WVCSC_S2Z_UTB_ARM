"""Thread-safe motion lock used by the arm task and motion-control node."""

import threading


class MotionControlState:
    """Keep stop/reset/resume semantics independent from ROS."""

    def __init__(self):
        self._mutex = threading.Lock()
        self._locked = False
        self._reset_in_progress = False

    @property
    def locked(self):
        with self._mutex:
            return self._locked

    @property
    def reset_in_progress(self):
        with self._mutex:
            return self._reset_in_progress

    def stop(self):
        with self._mutex:
            self._locked = True

    def begin_reset(self):
        """Lock motion and claim the single reset worker."""
        with self._mutex:
            self._locked = True
            if self._reset_in_progress:
                return False
            self._reset_in_progress = True
            return True

    def finish_reset(self):
        """A completed reset remains locked until an explicit resume."""
        with self._mutex:
            self._reset_in_progress = False

    def resume(self):
        """Unlock only after a reset worker has finished."""
        with self._mutex:
            if self._reset_in_progress:
                return False
            self._locked = False
            return True


def begin_reset(state, arm):
    """Claim reset and stop all currently owned motion."""
    if not state.begin_reset():
        return False
    if arm.cancel_and_wait():
        return True
    state.finish_reset()
    return False


def perform_reset(state, arm, home):
    """Open the gripper and plan HOME, always leaving reset mode locked."""
    try:
        if not arm.control_gripper(open_gripper=True, allow_locked=True):
            return False
        return bool(arm.move_joints(home, allow_locked=True))
    finally:
        state.finish_reset()

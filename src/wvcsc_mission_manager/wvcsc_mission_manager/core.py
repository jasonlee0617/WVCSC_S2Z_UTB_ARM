from dataclasses import dataclass
from enum import IntEnum
import math


class MissionState(IntEnum):
    IDLE = 0
    WAITING_FOR_TASKS = 1
    READY = 2
    NAVIGATING = 3
    VERIFYING_STOP = 4
    ARM_SPRAYING = 5
    TARGET_COMPLETED = 6
    PAUSED = 7
    MISSION_COMPLETED = 8
    CANCELED = 9
    FAILED = 10
    RETURNING_HOME = 11


@dataclass(frozen=True)
class Target:
    tree_id: str
    x: float
    y: float
    z: float
    confidence: float
    spray_side: str
    spray_duration: float
    evidence_uri: str = ''


class MissionCore:
    PENDING = 'PENDING'
    COMPLETED = 'COMPLETED'
    SKIPPED = 'SKIPPED'
    FAILED = 'FAILED'
    ACTIVE = {
        MissionState.NAVIGATING,
        MissionState.VERIFYING_STOP,
        MissionState.ARM_SPRAYING,
        MissionState.PAUSED,
        MissionState.RETURNING_HOME,
    }
    TERMINAL = {
        MissionState.MISSION_COMPLETED,
        MissionState.CANCELED,
        MissionState.FAILED,
    }

    def __init__(self):
        self.state = MissionState.WAITING_FOR_TASKS
        self.mission_id = ''
        self.targets = ()
        self.current_index = 0
        self.completed_targets = 0
        self.skipped_targets = 0
        self.target_outcomes = []
        self.last_error = ''

    @property
    def current_target(self):
        if 0 <= self.current_index < len(self.targets):
            return self.targets[self.current_index]
        return None

    def load(self, mission_id, targets):
        if mission_id and mission_id == self.mission_id:
            return 'duplicate'
        if self.state != MissionState.WAITING_FOR_TASKS:
            return 'busy'
        if not mission_id or not targets:
            raise ValueError('mission_id and targets are required')
        self.mission_id = mission_id
        self.targets = tuple(targets)
        self.current_index = 0
        self.completed_targets = 0
        self.skipped_targets = 0
        self.target_outcomes = [self.PENDING] * len(self.targets)
        self.last_error = ''
        self.state = MissionState.READY
        return 'accepted'

    def start(self):
        return self._transition(MissionState.READY, MissionState.NAVIGATING)

    def nav_succeeded(self):
        return self._transition(MissionState.NAVIGATING, MissionState.VERIFYING_STOP)

    def stop_verified(self):
        return self._transition(MissionState.VERIFYING_STOP, MissionState.ARM_SPRAYING)

    def arm_succeeded(self, return_home=False):
        if self.state != MissionState.ARM_SPRAYING:
            return False
        self.target_outcomes[self.current_index] = self.COMPLETED
        self.completed_targets += 1
        self.current_index += 1
        self.state = (
            MissionState.NAVIGATING
            if self.current_index < len(self.targets)
            else (
                MissionState.RETURNING_HOME
                if return_home else MissionState.MISSION_COMPLETED
            )
        )
        return True

    def skip_current(self, return_home=False):
        if self.state not in {
                MissionState.READY,
                MissionState.PAUSED,
                MissionState.VERIFYING_STOP,
                MissionState.ARM_SPRAYING}:
            return False
        previous_state = self.state
        self.target_outcomes[self.current_index] = self.SKIPPED
        self.current_index += 1
        self.skipped_targets += 1
        if self.current_index >= len(self.targets):
            self.state = (
                MissionState.RETURNING_HOME
                if return_home else MissionState.MISSION_COMPLETED
            )
        elif previous_state in {
                MissionState.VERIFYING_STOP,
                MissionState.ARM_SPRAYING}:
            self.state = MissionState.NAVIGATING
        else:
            self.state = previous_state
        return True

    def return_home(self):
        if self.state not in {
                MissionState.READY,
                MissionState.PAUSED,
                MissionState.VERIFYING_STOP}:
            return False
        self.state = MissionState.RETURNING_HOME
        return True

    def home_succeeded(self, canceled=False):
        target = (
            MissionState.CANCELED if canceled
            else MissionState.MISSION_COMPLETED)
        return self._transition(MissionState.RETURNING_HOME, target)

    def pause(self):
        return self._transition(MissionState.NAVIGATING, MissionState.PAUSED)

    def resume(self):
        return self._transition(MissionState.PAUSED, MissionState.NAVIGATING)

    def cancel(self):
        if self.state not in self.ACTIVE | {MissionState.READY}:
            return False
        self.state = MissionState.CANCELED
        return True

    def fail(self, message):
        if self.state in self.TERMINAL:
            return False
        previous_state = self.state
        self.last_error = str(message)
        if (previous_state != MissionState.RETURNING_HOME and
                self.current_index < len(self.target_outcomes) and
                self.target_outcomes[self.current_index] == self.PENDING):
            self.target_outcomes[self.current_index] = self.FAILED
        self.state = MissionState.FAILED
        return True

    def reset(self):
        if self.state not in self.TERMINAL:
            return False
        self.__init__()
        return True

    def _transition(self, expected, target):
        if self.state != expected:
            return False
        self.state = target
        return True


def docking_pose(target, road_center_y=0.0, road_yaw=0.0, standoff=1.5):
    if target.spray_side == 'left':
        if target.y <= road_center_y:
            raise ValueError(f'{target.tree_id}: left target is not left of the road')
        goal_y = target.y - standoff
    elif target.spray_side == 'right':
        if target.y >= road_center_y:
            raise ValueError(f'{target.tree_id}: right target is not right of the road')
        goal_y = target.y + standoff
    else:
        raise ValueError(f'{target.tree_id}: invalid spray_side')
    values = (target.x, goal_y, road_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{target.tree_id}: non-finite docking pose')
    return values


class StopDetector:
    WAITING = 'waiting'
    STABLE = 'stable'
    STALE = 'stale'
    TIMEOUT = 'timeout'

    def __init__(
            self, linear_threshold=0.03, angular_threshold=0.03,
            stable_duration=1.0, stale_timeout=1.0, timeout=5.0):
        self.linear_threshold = float(linear_threshold)
        self.angular_threshold = float(angular_threshold)
        self.stable_duration = float(stable_duration)
        self.stale_timeout = float(stale_timeout)
        self.timeout = float(timeout)
        self.active = False
        self.started_at = None
        self.last_update = None
        self.stable_since = None

    def start(self, now):
        self.active = True
        self.started_at = float(now)
        self.last_update = None
        self.stable_since = None

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
        freshness_origin = self.last_update if self.last_update is not None else self.started_at
        if now - freshness_origin >= self.stale_timeout:
            return self.STALE
        if self.stable_since is not None and now - self.stable_since >= self.stable_duration:
            return self.STABLE
        return self.WAITING

    def stop(self):
        self.active = False

from dataclasses import dataclass
from enum import IntEnum
import math


DEFAULT_DOCKING_LATERAL_OFFSET = 0.2


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
    docking_pose_override: tuple | None = None


class MissionCore:
    PENDING = 'PENDING'
    COMPLETED = 'COMPLETED'
    PARTIAL = 'PARTIAL'
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
        self.partial_targets = 0
        self.skipped_targets = 0
        self.target_outcomes = []
        self.target_messages = []
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
        self.partial_targets = 0
        self.skipped_targets = 0
        self.target_outcomes = [self.PENDING] * len(self.targets)
        self.target_messages = [''] * len(self.targets)
        self.last_error = ''
        self.state = MissionState.READY
        return 'accepted'

    def start(self):
        return self._transition(MissionState.READY, MissionState.NAVIGATING)

    def nav_succeeded(self):
        return self._transition(MissionState.NAVIGATING, MissionState.VERIFYING_STOP)

    def stop_verified(self):
        return self._transition(MissionState.VERIFYING_STOP, MissionState.ARM_SPRAYING)

    @property
    def processed_targets(self):
        return sum(outcome != self.PENDING for outcome in self.target_outcomes)

    @property
    def all_targets_completed(self):
        return bool(self.targets) and all(
            outcome == self.COMPLETED for outcome in self.target_outcomes)

    def _finish_after_current(self, return_home):
        if self.current_index < len(self.targets):
            self.state = MissionState.NAVIGATING
        elif return_home:
            self.state = MissionState.RETURNING_HOME
        elif self.all_targets_completed:
            self.state = MissionState.MISSION_COMPLETED
        else:
            self.state = MissionState.FAILED

    def arm_succeeded(self, return_home=False, message=''):
        if self.state != MissionState.ARM_SPRAYING:
            return False
        self.target_outcomes[self.current_index] = self.COMPLETED
        self.target_messages[self.current_index] = str(message)
        self.completed_targets += 1
        self.current_index += 1
        self._finish_after_current(return_home)
        return True

    def arm_partial(self, message='', return_home=False):
        """记录树级部分完成，并继续剩余树；最后必须以 FAILED 收尾。"""
        if self.state != MissionState.ARM_SPRAYING:
            return False
        self.target_outcomes[self.current_index] = self.PARTIAL
        self.target_messages[self.current_index] = str(message)
        self.partial_targets += 1
        self.last_error = (
            f'incomplete tree={self.targets[self.current_index].tree_id}: {message}')
        self.current_index += 1
        self._finish_after_current(return_home)
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
        self.target_messages[self.current_index] = 'tree skipped'
        self.last_error = (
            f'incomplete tree={self.targets[self.current_index].tree_id}: tree skipped')
        self.current_index += 1
        self.skipped_targets += 1
        if self.current_index >= len(self.targets):
            self._finish_after_current(return_home)
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
                MissionState.VERIFYING_STOP,
                MissionState.MISSION_COMPLETED}:
            return False
        self.state = MissionState.RETURNING_HOME
        return True

    def home_succeeded(self, canceled=False):
        target = (
            MissionState.CANCELED if canceled
            else (MissionState.MISSION_COMPLETED
                  if self.all_targets_completed else MissionState.FAILED))
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
            self.target_messages[self.current_index] = self.last_error
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


def docking_pose(
        target, road_center_y=0.0, road_yaw=0.0,
        lateral_offset=DEFAULT_DOCKING_LATERAL_OFFSET):
    values = (target.x, target.y, road_center_y, road_yaw, lateral_offset)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{target.tree_id}: non-finite docking pose')
    if lateral_offset < 0.0:
        raise ValueError('lateral_offset must be non-negative')
    if target.spray_side == 'left':
        if target.y <= road_center_y:
            raise ValueError(f'{target.tree_id}: left target is not left of the road')
        goal_y = road_center_y + lateral_offset
    elif target.spray_side == 'right':
        if target.y >= road_center_y:
            raise ValueError(f'{target.tree_id}: right target is not right of the road')
        goal_y = road_center_y - lateral_offset
    else:
        raise ValueError(f'{target.tree_id}: invalid spray_side')
    return target.x, goal_y, road_yaw


def navigation_pose(
        target, road_center_y=0.0, road_yaw=0.0,
        lateral_offset=DEFAULT_DOCKING_LATERAL_OFFSET):
    if target.docking_pose_override is not None:
        return target.docking_pose_override
    return docking_pose(target, road_center_y, road_yaw, lateral_offset)


def manual_tree_hint(docking, spray_side, standoff, tree_base_z=0.0):
    """Infer a tree-root point from an operator-selected docking pose."""
    x, y, yaw = (float(value) for value in docking)
    standoff = float(standoff)
    tree_base_z = float(tree_base_z)
    if not all(math.isfinite(value) for value in (x, y, yaw, standoff, tree_base_z)):
        raise ValueError('manual tree hint values must be finite')
    if standoff <= 0.0:
        raise ValueError('manual_tree_standoff must be positive')
    if spray_side not in ('left', 'right'):
        raise ValueError(f'invalid spray_side: {spray_side}')
    sign = 1.0 if spray_side == 'left' else -1.0
    normal_x, normal_y = -math.sin(yaw), math.cos(yaw)
    return (
        x + sign * standoff * normal_x,
        y + sign * standoff * normal_y,
        tree_base_z,
    )


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

# core.py
# ============================================================================
# WVCSC 任务编排核心状态机与数据类 (纯逻辑层，不依赖 ROS2)
# ============================================================================
#
# 职责：
# 1. 定义任务状态枚举 (`MissionState`)。
# 2. 管理任务队列、当前进度与目标状态流转 (`MissionCore`)。
# 3. 保存 Qt/RViz 给出的停车位，并提供车辆与机械臂基座坐标转换。
# 4. 提供里程计停稳检测器 (`StopDetector`)。
#
# 设计目的：
# 将 ROS2 通讯与核心业务逻辑解耦，便于进行单元测试，并降低状态异常转移的风险。
#

from dataclasses import dataclass
from enum import IntEnum
import math


DEFAULT_ARM_BASE_FORWARD_OFFSET = -0.40
DEFAULT_ARM_BASE_LEFT_OFFSET = 0.0


class MissionState(IntEnum):
    """任务执行状态枚举。"""
    IDLE = 0                # 空闲
    WAITING_FOR_TASKS = 1   # 等待任务列表注入
    READY = 2               # 已加载任务，等待“开始”指令
    NAVIGATING = 3          # 小车正在 Nav2 导航中
    VERIFYING_STOP = 4      # 小车到达目标点，正在检测是否停稳
    ARM_SPRAYING = 5        # 机械臂正在执行喷洒动作
    TARGET_COMPLETED = 6    # 单棵树处理完毕 (内部中转态)
    PAUSED = 7              # 任务暂停
    MISSION_COMPLETED = 8   # 全部任务成功完成
    CANCELED = 9            # 任务被取消
    FAILED = 10             # 任务严重失败
    RETURNING_HOME = 11     # 返回 HOME 位
    DWELLING = 12           # 路线点停留


class PointType(IntEnum):
    """Route point kind.  INSPECT is zero for legacy ManualMissionTarget."""
    INSPECT = 0
    TRANSIT = 1
    FINISH = 2


class WorkSide(IntEnum):
    """Declared tree side relative to alicia_base_link."""
    UNSPECIFIED = 0
    LEFT = 1
    RIGHT = 2


@dataclass(frozen=True)
class Target:
    """Qt/RViz 人工任务的固定数据；树根坐标仅由基座相对偏移派生。"""
    tree_id: str
    x: float
    y: float
    z: float
    spray_duration: float
    docking_pose: tuple
    tree_x_m: float = 0.0
    tree_y_m: float = 0.0
    point_type: int = PointType.INSPECT
    wide_spray_on_approach: bool = False
    dwell_time_sec: float = 0.0
    work_side: int = WorkSide.UNSPECIFIED

    @property
    def requires_arm(self):
        return int(self.point_type) == int(PointType.INSPECT)


class MissionCore:
    """线程安全的纯逻辑状态机。所有状态修改操作均应返回布尔值以指示是否成功。"""

    # 树级任务结果状态
    PENDING = 'PENDING'     # 未处理
    COMPLETED = 'COMPLETED' # 成功喷洒
    PARTIAL = 'PARTIAL'     # 部分喷洒/部分成功
    SKIPPED = 'SKIPPED'     # 被主动跳过
    FAILED = 'FAILED'       # 执行失败

    # 活跃状态集合 (允许在这些状态下执行取消操作)
    ACTIVE = {
        MissionState.NAVIGATING,
        MissionState.VERIFYING_STOP,
        MissionState.ARM_SPRAYING,
        MissionState.PAUSED,
        MissionState.RETURNING_HOME,
        MissionState.DWELLING,
    }
    # 终端状态集合 (任务已彻底结束)
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
        """返回当前正在处理的目标树。"""
        if 0 <= self.current_index < len(self.targets):
            return self.targets[self.current_index]
        return None

    def load(self, mission_id, targets):
        """加载任务列表，仅当状态为 WAITING_FOR_TASKS 时才接受。"""
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
        if self.state != MissionState.VERIFYING_STOP:
            return False
        target = self.current_target
        if target is None:
            return False
        self.state = (
            MissionState.ARM_SPRAYING if target.requires_arm
            else MissionState.DWELLING)
        return True

    def retry_navigation(self):
        """停靠质量不合格时重新导航当前目标，不推进任务索引。"""
        return self._transition(MissionState.VERIFYING_STOP, MissionState.NAVIGATING)

    @property
    def processed_targets(self):
        return sum(outcome != self.PENDING for outcome in self.target_outcomes)

    @property
    def all_targets_completed(self):
        return bool(self.targets) and all(
            outcome == self.COMPLETED for outcome in self.target_outcomes)

    @property
    def all_targets_processed(self):
        return bool(self.targets) and all(
            outcome != self.PENDING for outcome in self.target_outcomes)

    def _finish_after_current(self, return_home):
        """处理完当前任务后，决定是否进入下一棵树、返回 Home 或完成任务。"""
        if self.current_index < len(self.targets):
            self.state = MissionState.NAVIGATING
        elif return_home:
            self.state = MissionState.RETURNING_HOME
        elif self.all_targets_processed:
            self.state = MissionState.MISSION_COMPLETED
        else:
            self.state = MissionState.FAILED

    def point_succeeded(self, return_home=False, message=''):
        """Finish a transit or finish point after its optional dwell."""
        if self.state != MissionState.DWELLING:
            return False
        self.target_outcomes[self.current_index] = self.COMPLETED
        self.target_messages[self.current_index] = str(message)
        self.completed_targets += 1
        self.current_index += 1
        self._finish_after_current(return_home)
        return True

    def arm_succeeded(self, return_home=False, message=''):
        """机械臂喷洒成功。"""
        if self.state != MissionState.ARM_SPRAYING:
            return False
        self.target_outcomes[self.current_index] = self.COMPLETED
        self.target_messages[self.current_index] = str(message)
        self.completed_targets += 1
        self.current_index += 1
        self._finish_after_current(return_home)
        return True

    def arm_partial(self, message='', return_home=False):
        """记录树级部分成功；路线继续，最终状态仍可正常结束。"""
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

    def skip_current(self, return_home=False, message='tree skipped'):
        """跳过当前任务。"""
        if self.state not in {
                MissionState.READY,
                MissionState.PAUSED,
                MissionState.NAVIGATING,
                MissionState.VERIFYING_STOP,
                MissionState.ARM_SPRAYING,
                MissionState.DWELLING}:
            return False
        previous_state = self.state
        self.target_outcomes[self.current_index] = self.SKIPPED
        self.target_messages[self.current_index] = str(message)
        self.last_error = (
            f'incomplete tree={self.targets[self.current_index].tree_id}: {message}')
        self.current_index += 1
        self.skipped_targets += 1
        if self.current_index >= len(self.targets):
            self._finish_after_current(return_home)
        elif previous_state in {
                MissionState.VERIFYING_STOP,
                MissionState.ARM_SPRAYING,
                MissionState.NAVIGATING,
                MissionState.DWELLING}:
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
            else MissionState.MISSION_COMPLETED)
        return self._transition(MissionState.RETURNING_HOME, target)

    def pause(self):
        if self.state not in {
                MissionState.NAVIGATING}:
            return False
        self.state = MissionState.PAUSED
        return True

    def pause_for_recovery(self):
        if self.state not in {
                MissionState.NAVIGATING,
                MissionState.VERIFYING_STOP,
                MissionState.RETURNING_HOME}:
            return False
        self.state = MissionState.PAUSED
        return True

    def resume(self, returning_home=False):
        if self.state != MissionState.PAUSED:
            return False
        self.state = (
            MissionState.RETURNING_HOME if returning_home
            else MissionState.NAVIGATING)
        return True

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


def arm_base_xy(
        vehicle_pose,
        arm_base_forward_offset=DEFAULT_ARM_BASE_FORWARD_OFFSET,
        arm_base_left_offset=DEFAULT_ARM_BASE_LEFT_OFFSET):
    """Return the map XY position of ``alicia_base_link``.

    ``vehicle_pose`` is an ``(x, y, yaw)`` pose for ``base_footprint``.  The
    yaw remains the vehicle heading; only the fixed planar mounting offset is
    applied here.
    """
    x, y, yaw = (float(value) for value in vehicle_pose)
    if not all(math.isfinite(value) for value in (
            x, y, yaw, arm_base_forward_offset, arm_base_left_offset)):
        raise ValueError('arm base pose values must be finite')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        x + cosine * arm_base_forward_offset - sine * arm_base_left_offset,
        y + sine * arm_base_forward_offset + cosine * arm_base_left_offset,
    )


def tree_hint_from_arm_base_offset(
        docking, tree_x_m, tree_y_m, tree_base_z=0.0,
        arm_base_forward_offset=DEFAULT_ARM_BASE_FORWARD_OFFSET,
        arm_base_left_offset=DEFAULT_ARM_BASE_LEFT_OFFSET,
        arm_base_yaw_rad=0.0):
    """Transform a signed alicia_base_link XY offset into a map tree hint."""
    x, y, yaw = (float(value) for value in docking)
    tree_x_m = float(tree_x_m)
    tree_y_m = float(tree_y_m)
    tree_base_z = float(tree_base_z)
    if not all(math.isfinite(value) for value in (
            x, y, yaw, tree_x_m, tree_y_m, tree_base_z,
            arm_base_forward_offset, arm_base_left_offset,
            arm_base_yaw_rad)):
        raise ValueError('tree offset values must be finite')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    arm_x = (
        x + cosine * arm_base_forward_offset -
        sine * arm_base_left_offset)
    arm_y = (
        y + sine * arm_base_forward_offset +
        cosine * arm_base_left_offset)
    arm_yaw = yaw + arm_base_yaw_rad
    arm_cosine, arm_sine = math.cos(arm_yaw), math.sin(arm_yaw)
    return (
        arm_x + arm_cosine * tree_x_m - arm_sine * tree_y_m,
        arm_y + arm_sine * tree_x_m + arm_cosine * tree_y_m,
        tree_base_z,
    )


def tree_offset_from_docking(
        docking, tree_hint,
        arm_base_forward_offset=DEFAULT_ARM_BASE_FORWARD_OFFSET,
        arm_base_left_offset=DEFAULT_ARM_BASE_LEFT_OFFSET,
        arm_base_yaw_rad=0.0):
    """Return signed alicia_base_link XY coordinates for a map tree hint."""
    x, y, yaw = (float(value) for value in docking)
    tree_x, tree_y, _tree_z = (float(value) for value in tree_hint)
    if not all(math.isfinite(value) for value in (
            x, y, yaw, tree_x, tree_y,
            arm_base_forward_offset, arm_base_left_offset,
            arm_base_yaw_rad)):
        raise ValueError('tree offset values must be finite')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    arm_x = (
        x + cosine * arm_base_forward_offset -
        sine * arm_base_left_offset)
    arm_y = (
        y + sine * arm_base_forward_offset +
        cosine * arm_base_left_offset)
    arm_yaw = yaw + arm_base_yaw_rad
    arm_cosine, arm_sine = math.cos(arm_yaw), math.sin(arm_yaw)
    dx, dy = tree_x - arm_x, tree_y - arm_y
    return (
        arm_cosine * dx + arm_sine * dy,
        -arm_sine * dx + arm_cosine * dy,
    )


class StopDetector:
    """
    小车停稳检测器。
    
    作用：根据 /odom 发来的线速度和角速度，判断小车是否已经完全静止。
    这是一个严格的物理“刹车”检测，必须持续 `stable_duration` (1s) 都满足阈值，
    才允许机械臂展开喷洒。这防止了小车在还没停稳的惯性滑动阶段机械臂意外展开。
    """
    WAITING = 'waiting'
    STABLE = 'stable'
    STALE = 'stale'    # 里程计数据长时间未更新 (断连)
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

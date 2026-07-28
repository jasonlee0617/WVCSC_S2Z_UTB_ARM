# core.py
# ============================================================================
# WVCSC 任务编排核心状态机与数据类 (纯逻辑层，不依赖 ROS2)
# ============================================================================
#
# 职责：
# 1. 定义任务状态枚举 (`MissionState`)。
# 2. 管理任务队列、当前进度与目标状态流转 (`MissionCore`)。
# 3. 保存 Qt/RViz 给出的停车位，并提供车辆与机械臂基座坐标转换。
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
    PAUSED = 7              # 任务暂停
    MISSION_COMPLETED = 8   # 全部任务成功完成
    CANCELED = 9            # 任务被取消
    FAILED = 10             # 任务严重失败
    RETURNING_HOME = 11     # 返回 HOME 位
    DWELLING = 12           # 路线点停留


class PointType(IntEnum):
    """Route point kind. INSPECT remains zero for default-initialized points."""
    INSPECT = 0
    TRANSIT = 1


class WorkSide(IntEnum):
    """Declared tree side relative to alicia_base_link."""
    UNSPECIFIED = 0
    LEFT = 1
    RIGHT = 2


@dataclass(frozen=True)
class RoutePoint:
    """Qt/RViz 人工任务的固定路线点；树根坐标仅由基座相对偏移派生。"""
    point_id: str
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
        self.points = ()
        self.current_index = 0
        self.completed_points = 0
        self.partial_points = 0
        self.skipped_points = 0
        self.point_outcomes = []
        self.point_messages = []
        self.last_error = ''

    @property
    def current_point(self):
        """返回当前正在处理的路线点。"""
        if 0 <= self.current_index < len(self.points):
            return self.points[self.current_index]
        return None

    def load(self, mission_id, points):
        """加载任务列表，仅当状态为 WAITING_FOR_TASKS 时才接受。"""
        if mission_id and mission_id == self.mission_id:
            return 'duplicate'
        if self.state != MissionState.WAITING_FOR_TASKS:
            return 'busy'
        if not mission_id or not points:
            raise ValueError('mission_id and points are required')
        self.mission_id = mission_id
        self.points = tuple(points)
        self.current_index = 0
        self.completed_points = 0
        self.partial_points = 0
        self.skipped_points = 0
        self.point_outcomes = [self.PENDING] * len(self.points)
        self.point_messages = [''] * len(self.points)
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
        point = self.current_point
        if point is None:
            return False
        self.state = (
            MissionState.ARM_SPRAYING if point.requires_arm
            else MissionState.DWELLING)
        return True

    @property
    def processed_points(self):
        return sum(outcome != self.PENDING for outcome in self.point_outcomes)

    @property
    def all_points_completed(self):
        return bool(self.points) and all(
            outcome == self.COMPLETED for outcome in self.point_outcomes)

    @property
    def all_points_processed(self):
        return bool(self.points) and all(
            outcome != self.PENDING for outcome in self.point_outcomes)

    def _finish_after_current(self, return_home):
        """处理完当前任务后，决定是否进入下一棵树、返回 Home 或完成任务。"""
        if self.current_index < len(self.points):
            self.state = MissionState.NAVIGATING
        elif return_home:
            self.state = MissionState.RETURNING_HOME
        elif self.all_points_processed:
            self.state = MissionState.MISSION_COMPLETED
        else:
            self.state = MissionState.FAILED

    def point_succeeded(self, return_home=False, message=''):
        """Finish a transit point after its optional dwell."""
        if self.state != MissionState.DWELLING:
            return False
        self.point_outcomes[self.current_index] = self.COMPLETED
        self.point_messages[self.current_index] = str(message)
        self.completed_points += 1
        self.current_index += 1
        self._finish_after_current(return_home)
        return True

    def arm_succeeded(self, return_home=False, message=''):
        """机械臂喷洒成功。"""
        if self.state != MissionState.ARM_SPRAYING:
            return False
        self.point_outcomes[self.current_index] = self.COMPLETED
        self.point_messages[self.current_index] = str(message)
        self.completed_points += 1
        self.current_index += 1
        self._finish_after_current(return_home)
        return True

    def arm_partial(self, message='', return_home=False):
        """记录树级部分成功；路线继续，最终状态仍可正常结束。"""
        if self.state != MissionState.ARM_SPRAYING:
            return False
        self.point_outcomes[self.current_index] = self.PARTIAL
        self.point_messages[self.current_index] = str(message)
        self.partial_points += 1
        self.last_error = (
            f'incomplete point={self.points[self.current_index].point_id}: {message}')
        self.current_index += 1
        self._finish_after_current(return_home)
        return True

    def skip_current(self, return_home=False, message='point skipped'):
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
        self.point_outcomes[self.current_index] = self.SKIPPED
        self.point_messages[self.current_index] = str(message)
        self.last_error = (
            f'incomplete point={self.points[self.current_index].point_id}: {message}')
        self.current_index += 1
        self.skipped_points += 1
        if self.current_index >= len(self.points):
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
                self.current_index < len(self.point_outcomes) and
                self.point_outcomes[self.current_index] == self.PENDING):
            self.point_outcomes[self.current_index] = self.FAILED
            self.point_messages[self.current_index] = self.last_error
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

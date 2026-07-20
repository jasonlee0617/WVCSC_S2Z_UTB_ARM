# core.py
"""
喷洒执行器的核心安全互锁模块 (Spray Interlock)。

本模块定义了喷洒泵/阀控制器的状态锁（State Lock）和紧急停止仲裁。
它不直接执行硬件动作，而是作为 Spray Action Server (spray_simulator.py)
和未来真机驱动的一个纯逻辑状态层，保证在并发环境下，喷洒动作具有明确的
独占性、安全性和防误触机制。

采用多线程互斥锁 (`threading.Lock`) 保证状态操作的原子性。
"""

import math
import threading


class SprayInterlock:
    """
    喷洒安全互锁类。

    职责：
    1. 验证喷洒目标的参数合法性（如时长是否在范围内）。
    2. 确保同一时间只有一个喷洒目标占据控制权（独占锁）。
    3. 响应系统层面的急停信号，强制切断喷洒执行权限。
    """

    def __init__(self, min_duration=0.2, max_duration=10.0):
        """
        初始化互锁状态。

        Args:
            min_duration (float): 允许的最小喷洒时长（秒），防止异常短脉冲。
            max_duration (float): 允许的最大喷洒时长（秒），防止逻辑死锁或爆管。
        """
        self.min_duration = float(min_duration)
        self.max_duration = float(max_duration)
        self._lock = threading.Lock()   # 线程安全锁，保护内部状态变量
        self._active = False            # 当前是否正在执行喷洒任务
        self._emergency_stopped = False # 全局紧急停止状态

    @property
    def active(self):
        """返回当前喷洒执行器是否被占用。"""
        with self._lock:
            return self._active

    @property
    def emergency_stopped(self):
        """返回系统是否处于紧急停止锁定状态。"""
        with self._lock:
            return self._emergency_stopped

    def validate(self, mission_id, tree_id, duration, mode):
        """
        喷洒动作合法性校验。

        在进入状态机之前，对 Action 请求的参数进行安全把关。
        这在 ROS2 Action 的 `goal_callback` 阶段被调用，用于及早拒绝无效请求。

        Args:
            mission_id (str): 任务 ID。
            tree_id (str): 树 ID。
            duration (float): 喷洒请求的持续时间。
            mode (str): 喷洒模式（当前仅支持 "continuous"）。

        Returns:
            str: 如果校验通过，返回空字符串 ''；如果失败，返回具体的错误原因。
        """
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
        """
        尝试获取喷洒执行器的控制权（加锁）。

        这是一个原子操作（由互斥锁保护）。只有同时满足“未激活”且“未急停”
        两个条件时，才能成功获取控制权并返回 True。

        Returns:
            bool: 如果成功获取喷洒控制权，返回 True；否则返回 False。
        """
        with self._lock:
            if self._active or self._emergency_stopped:
                return False
            self._active = True
            return True

    def release(self):
        """
        释放喷洒执行器的控制权（解锁）。

        必须在喷洒动作执行完毕（无论成功或失败）后调用，以允许后续任务进入。
        """
        with self._lock:
            self._active = False

    def set_emergency_stop(self, active):
        """
        设置全局紧急停止状态。

        该状态一旦设为 True，将导致后续所有 `claim()` 请求失败。
        通常由订阅 `/safety/emergency_stop` 话题的订阅者调用。
        若要恢复系统，需由上层安全逻辑将 active 设为 False。

        Args:
            active (bool): True 表示触发紧急停止，False 表示解除紧急停止。
        """
        with self._lock:
            self._emergency_stopped = bool(active)
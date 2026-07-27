# motion_state.py
"""
线程安全的运动控制状态机 (Motion Control State)。

本项目中的 `wvcsc_motion_control` 节点和 `wvcsc_spray_task` 节点通过此状态机进行
安全仲裁。它独立于 ROS2 通讯层，使用纯 Python 的 threading 模块实现互斥锁控制，
确保在 `MultiThreadedExecutor` 并发环境下，状态变更的原子性和可见性。

状态机流转规则：
1. 正常执行: `_locked = False`, `_reset_in_progress = False` -> 允许运动。
2. 急停 (Stop): 设定 `_locked = True` -> 阻断所有新任务，底层运动立即取消。
3. 复位 (Reset): 设定 `_locked = True`, `_reset_in_progress = True` -> 阻断所有任务，
                并在后台线程执行机械臂回 HOME 的物理动作。
4. 恢复 (Resume): 仅当 `_reset_in_progress = False` 时允许执行，重置 `_locked = False`
                  -> 允许系统重新接收任务。
"""

import threading


class MotionControlState:
    """保持停止/复位/恢复语义，与 ROS 基础设施解耦的线程安全状态机。"""

    def __init__(self):
        # 保护内部状态变量的互斥锁
        self._mutex = threading.Lock()
        # 锁定标志位：`True` 表示当前机械臂被强制锁死，不接受任何新任务
        self._locked = False
        # 复位中标志位：`True` 表示后台正处于执行 `reset` 回到 HOME 的物理运动流程中
        self._reset_in_progress = False

    @property
    def locked(self):
        """返回当前是否处于锁定状态（拒绝下游任务）。"""
        with self._mutex:
            return self._locked

    @property
    def reset_in_progress(self):
        """返回当前是否正在执行回到 HOME 的复位动作。"""
        with self._mutex:
            return self._reset_in_progress

    def stop(self):
        """
        【紧急停止】动作。
        
        立即锁定系统，不允许任何新的运动生成。
        实际底层的运动取消需要结合 `alicia_moveit.cancel()` 一起执行。
        """
        with self._mutex:
            self._locked = True

    def begin_reset(self):
        """
        发起复位流程的起始阶段（原子操作）。

        将系统立即锁定，并标记复位动作已经开始。
        返回布尔值表明是否成功获取了“复位许可”（同一时间只允许一个复位线程执行）。
        """
        with self._mutex:
            # 如果系统已经被锁定，或者已经有另一个重置线程在执行，则不允许发起新的复位
            self._locked = True
            if self._reset_in_progress:
                return False
            self._reset_in_progress = True
            return True

    def finish_reset(self):
        """
        复位流程的结束阶段（无论成功还是失败，都必须调用）。

        仅将 `_reset_in_progress` 置为 False。
        **注意**：系统依然保持 `_locked = True` 状态，强制要求外部发送 `resume` 指令
        后才能继续执行任务，这符合“人工确认安全后恢复”的工业安全原则。
        """
        with self._mutex:
            self._reset_in_progress = False

    def resume(self):
        """
        【恢复】动作。

        解除系统的锁定状态，允许新的运动任务下发。
        安全约束：只有在复位动作完全结束（`_reset_in_progress = False`）后，才能恢复。
        """
        with self._mutex:
            if self._reset_in_progress:
                return False
            self._locked = False
            return True


def begin_reset(state, arm):
    """
    复位动作的前置安全拦截与停止函数（由外部 `motion_control` 线程调用）。
    
    1. 尝试获取状态机的复位锁 (`state.begin_reset()`)。
    2. 立刻取消当前正在执行的所有机械臂和夹爪运动 (`arm.cancel_and_wait()`)。
    3. 只有成功获取锁，并且底层的运动被完全停止，才返回 True。
    """
    # 尝试申请复位锁。如果申请失败（如已存在其他复位线程），直接返回 False。
    if not state.begin_reset():
        return False
    
    # 强制取消当前的机械臂运动，并阻塞等待直到底盘/机械臂确认进入 IDLE 空闲状态。
    # 如果取消失败或等待超时，调用 `state.finish_reset()` 还原状态，并返回 False。
    if arm.cancel_and_wait():
        return True
    
    # 如果取消等待失败，必须要调用 finish_reset，否则状态机将永远卡在 `_reset_in_progress`，
    # 导致系统永远无法执行 `resume` 解开锁。
    state.finish_reset()
    return False


def perform_reset(state, arm, home, abort_requested=None):
    """
    执行实际的物理复位动作（在后台线程中运行）。
    
    1. 打开夹爪（保持在安全开合状态）。
    2. 规划并执行回 HOME 点的关节空间轨迹。
    3. **无论成功或失败**，在退出前都必须调用 `state.finish_reset()` 以标记复位流程结束。
    
    注意：物理执行结束后，状态机仍被锁定 (`_locked=True`)，需要等待外部手动发送 `resume`。
    """
    try:
        if abort_requested is not None and abort_requested():
            return False
        # 1. 打开夹爪，防止复位过程中夹爪捏合导致意外损坏或卡住。
        if not arm.control_gripper(open_gripper=True, allow_locked=True):
            return False
        if abort_requested is not None and abort_requested():
            return False
        # 2. 规划并执行机械臂回到 HOME 的关节空间运动
        # 使用 `allow_locked=True` 使得系统即使在 `_locked=True` 的情况下，依然能够执行此次复位运动。
        return bool(arm.move_joints(home, allow_locked=True))
    finally:
        # `finally` 块确保了无论上面的逻辑是否报错或返回 False，
        # `_reset_in_progress` 都会得到重置，否则整个机械臂将永久死锁。
        state.finish_reset()

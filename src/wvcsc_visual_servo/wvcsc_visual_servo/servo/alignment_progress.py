# alignment_progress.py
"""
视觉伺服对准进度状态评估器 (Alignment Progress Evaluator)。

职责：
1. 测量当前目标的像素误差是否满足“精细对准”阈值 (`fine_tolerance_px`)。
2. 强制要求目标在精细对准区内连续稳定停留足够长的时间 (`stable_duration_sec`)，
   以防止因单帧偶然噪点导致虚假成功。
3. 实现控制迟滞带 (Hysteresis)，允许YOLO掩膜在像素级轻微抖动，而不会中断稳定计时。
4. 检测伺服控制的“卡死”状态（误差在给定时间内没有显著减小），用于触发超时恢复机制。
"""

import math


class AlignmentProgress:
    """
    基于时间的收敛与有效误差缩减跟踪器。
    
    该类与 PID 控制器分离，独立评估视觉伺服的最终结果，避免控制计算逻辑与
    成功判定逻辑混杂在一起。
    """

    def __init__(
            self, fine_tolerance_px, stable_duration_sec,
            progress_window_sec, min_progress_px,
            stable_reset_tolerance_px=None):
        """
        初始化对准进度评估器。

        Args:
            fine_tolerance_px (float): 目标进入的精细对准阈值（像素）。例如 1.5。
            stable_duration_sec (float): 目标在精细对准区内必须保持的最小稳定时间（秒）。例如 0.5。
            progress_window_sec (float): 判定“进度停滞”的时间窗口（秒）。例如 4.0。
            min_progress_px (float): 在时间窗口内，目标必须缩小的最小像素误差。例如 1.0。
            stable_reset_tolerance_px (float, optional): 控制迟滞带的退出阈值。
                如果未指定，默认为 `fine_tolerance_px`。
        """
        self.fine_tolerance_px = float(fine_tolerance_px)
        self.stable_reset_tolerance_px = float(
            stable_reset_tolerance_px
            if stable_reset_tolerance_px is not None else fine_tolerance_px)
        self.stable_duration_sec = float(stable_duration_sec)
        self.progress_window_sec = float(progress_window_sec)
        self.min_progress_px = float(min_progress_px)
        self.reset()

    def reset(self):
        """完全重置所有状态，准备接收新的目标。"""
        self._stable_since = None      # 开始进入细容差区的时间戳
        self._stable_last = None       # 最后一次满足容差条件的时间戳
        self._progress_since = None    # 上一次观察到明显进步的时间戳
        self._progress_reference = None # 上一次明显进步时的误差参考值
        self._last_norm = math.inf     # 最近一帧的误差范数

    def reset_stable(self):
        """仅重置稳定状态，保留进度检查状态。

        在目标重新捕获或重新出现时调用，清空满足“稳定”的时间累积。
        """
        self._stable_since = None
        self._stable_last = None

    def restart_progress(self, error_u_px, error_v_px, now):
        """在目标重新捕获后，重新启动进度看门狗。

        为了确保目标在丢失后重新捕获时，不被之前旧的滞后的“进度卡顿”所影响，
        强制将当前的误差作为新的参考基准。
        """
        self._progress_since = float(now)
        self._progress_reference = math.hypot(
            float(error_u_px), float(error_v_px))

    def update(self, error_u_px, error_v_px, now):
        """
        根据新一帧的像素误差更新时间窗口。

        Returns:
            float: 当前最新的误差范数 (Euclidean norm)。
        """
        now = float(now)
        norm = math.hypot(float(error_u_px), float(error_v_px))
        self._last_norm = norm

        # ---------------- 1. 稳定时间计算 ----------------
        # 必须使用欧氏范数来验证是否满足精细公差。
        # 如果只检查 XY 各轴是否小于 1.5，那么当误差为 (1.9, 1.4) px 时，
        # 实际欧氏误差为 2.36 px，大于要求的 1.5 px，这会导致假阳性。
        within_tolerance = norm <= self.fine_tolerance_px

        if within_tolerance:
            # 如果本次进入精细容差：
            if self._stable_since is None:
                self._stable_since = now
            self._stable_last = now
        elif (self._stable_since is not None and
              norm <= self.stable_reset_tolerance_px):
            # 【迟滞带判定】(Hysteresis)
            # YOLO 分割掩膜中心在 1.5~2.0 px 之间容易发生小幅度像素抖动。
            # 此时如果误差仅略大于 1.5 px，我们不重置 `_stable_since`，
            # 而是**继续维持已有的稳定窗口**。
            # 只要误差没有超过 `stable_reset_tolerance_px`（例如 2.0 px），
            # 系统就认为依然处于“靠近收敛”的状态，保持计时。
            self._stable_last = now
        else:
            # 如果误差严重跳出容差和迟滞带，完全重置稳定状态。
            self.reset_stable()

        # ---------------- 2. 进度看门狗 ----------------
        # 如果当前的误差范数相比上次显著进步减少超过 `min_progress_px` 像素，
        # 说明伺服正在有效工作，重置卡顿计时器。
        if self._progress_since is None:
            self._progress_since = now
            self._progress_reference = norm
        elif self._progress_reference - norm >= self.min_progress_px:
            self._progress_since = now
            self._progress_reference = norm
        
        return norm

    @property
    def stable_duration(self):
        """返回当前已知的稳定累积时间（秒）。"""
        if self._stable_since is None or self._stable_last is None:
            return 0.0
        return max(0.0, self._stable_last - self._stable_since)

    @property
    def aligned(self):
        """返回是否满足对准成功条件。

        必须同时满足：
        1. 当前误差严格小于精细容差。
        2. 稳定持续时间大于等于配置要求。
        """
        return (
            self._last_norm <= self.fine_tolerance_px
            and self.stable_duration >= self.stable_duration_sec)

    def stalled(self, now):
        """返回视觉伺服是否陷入卡顿（Stall）。

        如果在 `progress_window_sec` (例如 4秒) 内，误差无法减少超过 `min_progress_px`，
        则认为 PID 进入了卡顿状态（例如被卡在奇异点，或目标完全静止不动）。
        此时上层（如 `spray_task`）需要触发超时和重新定位。
        """
        return (
            self._progress_since is not None
            and float(now) - self._progress_since >= self.progress_window_sec)
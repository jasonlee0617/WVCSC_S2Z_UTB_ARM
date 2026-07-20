# pid_controller.py
"""
二维图像平面 PID 控制器 (2D PID Controller for Visual Servoing)。

职责：
1. 接收当前目标在图像平面上的像素误差 (error_u, error_v)。
2. 分别对 U 轴和 V 轴误差执行 PID 计算。
3. 通过 Kd 项的低通滤波（EMA 平滑）消除 YOLO 分割掩膜带来的单帧像素抖动。
4. 输出限制后的图像平面速度指令 (Twist)，供后续给机械臂执行。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ServoControlConfig:
    """
    伺服 PID 控制器配置参数集合。
    所有参数均从 `visual_servo.yaml` 解析注入。
    """
    kp_xy: float = 0.25          # 比例增益 (Proportional)
    ki_xy: float = 0.0           # 积分增益 (Integral，视觉伺服通常为0)
    kd_xy: float = 0.01          # 微分增益 (Derivative)
    d_ema_alpha: float = 0.65    # 微分项的指数移动平均平滑系数 (0~1)
    derivative_clip_xy: float = 2.0   # 导数限幅阈值，防止单帧像素跳变导致速度突变
    integral_limit_xy: float = 0.10  # 积分限幅，限制抗积分饱和的最大累积量


def _clamp(value, limit):
    """将数值限制在 [-limit, limit] 范围内。"""
    return max(-limit, min(limit, value))


class PIDController2D:
    """
    基于 RGB 图像的二维视觉伺服 PID 控制器。

    核心特征：
    - 双轴独立控制：虽然计算公式一致，但 U轴 和 V轴 独立计算误差并独立输出。
    - 微分项平滑：使用一阶低通滤波器（EMA）处理导数，解决 YOLO 掩膜中心的像素级高频抖动问题。
    """

    def __init__(self, config):
        self.cfg = config
        self.reset()

    def reset(self):
        """重置控制器的内部历史状态，准备处理新目标。"""
        self._last = None          # 上一帧的误差值 (用于计算 d_error)
        self._derivative = (0.0, 0.0)   # 平滑后的 D 项缓存
        self._integral = (0.0, 0.0)     # 积分项缓存

    def step(self, error, dt):
        """
        核心执行步骤：输入当前像素误差和时间步长，输出速度指令。

        核心逻辑：
        1. 计算原始微分 (raw_derivative)。
        2. 使用 EMA 平滑微分项，消除单帧噪声。
        3. 计算积分项，并应用抗积分饱和限幅。
        4. 合成 PID 输出。

        Args:
            error (tuple): (error_u, error_v) 图像平面的像素误差。
            dt (float): 距离上一次调用的时间步长 (秒)。

        Returns:
            tuple: (command_u, command_v, debug_dict)
                - command_u: 图像平面 U 轴期望线速度 (归一化数值)。
                - command_v: 图像平面 V 轴期望线速度 (归一化数值)。
                - debug_dict: 包含当前 P/I/D 各项拆解数值的字典，用于高频调试。
        """
        # 1. 保护性检查：防止 dt 过小导致微分爆炸
        dt = max(1e-3, float(dt))
        error = (float(error[0]), float(error[1]))

        # 2. 原始微分计算
        if self._last is None:
            raw = (0.0, 0.0)
        else:
            # (当前误差 - 上次误差) / dt 得到微分，并通过 derivative_clip 限幅
            raw = tuple(
                _clamp((value - previous) / dt, self.cfg.derivative_clip_xy)
                for value, previous in zip(error, self._last)
            )

        # 3. 指数移动平均 (EMA) 滤波
        # 机器人视觉伺服领域经典的"死穴"：YOLO 的像素级抖动（约 0.5~1px 噪声）。
        # 直接使用原始微分会在 PID 输出中引入高频震颤。
        # 通过 d_ema_alpha 结合上一帧的平滑导数，能够有效滤除高频噪声。
        alpha = float(self.cfg.d_ema_alpha)
        derivative = tuple(
            alpha * value + (1.0 - alpha) * previous
            for value, previous in zip(raw, self._derivative)
        )

        # 4. 积分项计算与抗积分饱和 (Anti-windup)
        # 将误差累加，并限制在 integral_limit_xy 范围内，防止系统长时间失控时积分无限增大。
        integral = tuple(
            _clamp(total + value * dt, self.cfg.integral_limit_xy)
            for total, value in zip(self._integral, error)
        )

        # 5. 合成 PID 输出
        command = tuple(
            self.cfg.kp_xy * value          # P 项
            + self.cfg.ki_xy * total        # I 项
            + self.cfg.kd_xy * rate         # D 项
            for value, total, rate in zip(error, integral, derivative)
        )

        # 6. 更新内部状态供下一帧使用
        self._last = error
        self._derivative = derivative
        self._integral = integral

        # 返回控制量及用于调试的拆解数据
        return command[0], command[1], {
            'error': error,
            'derivative': derivative,
            'integral': integral,
        }
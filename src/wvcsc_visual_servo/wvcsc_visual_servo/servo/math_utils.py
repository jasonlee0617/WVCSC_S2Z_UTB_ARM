# math_utils.py
"""
视觉伺服 (IBVS) 控制回路共享的纯数学工具模块。

职责：
1. 限制控制回路的时间步长 (dt)，防止数值积分爆炸。
2. 实现速率限制 (Slew Rate Limiting)，保护机械臂免受 PID 输出突变导致的冲击。
3. 按比例缩放 XY 速度范数，防止超过最大物理速度阈值。
4. 提供二维目标的简易线性预测器，补偿视觉感知与机械臂反应之间的微小延迟。
"""

import math


def bounded_control_dt(elapsed, control_rate_hz):
    """
    限制控制回路的时间步长 (dt)。

    `visual_servo_node.py` 使用 `time.monotonic()` 记录真实墙钟时间计算 dt，
    但在 Gazebo 仿真中，/clock 可能会比真实时间慢（或滞后）。
    如果 dt 过小，会导致微分项除零或积分不更新；
    如果 dt 过大，会导致 PID 积过分累计，引发机械臂剧烈震荡。

    Args:
        elapsed (float): 距离上次控制循环的真实时间间隔 (秒)。
        control_rate_hz (float): 目标控制频率 (如 30.0 Hz)。

    Returns:
        float: 经过安全裁剪的 dt，范围限制在 [0.001, 2 * 1/control_rate_hz] 内。
    """
    period = 1.0 / float(control_rate_hz)
    # 限制 dt 最小为 1ms (防止除零)，最大为 2个控制周期 (防止偶然的仿真卡顿导致大幅超调)
    return max(1e-3, min(2.0 * period, float(elapsed)))


def slew(value, previous, acceleration, dt):
    """
    速率限制器 (Slew Rate Limiting)。

    将当前的期望输出 (value) 与前一次的实际输出 (previous) 进行比较。
    如果在单个时间步 (dt) 内，变化量超过了物理加速度限制 (acceleration)，
    则将变化量裁剪到最大允许值。

    这是工业控制中极其重要的安全防护：它防止 PID 的单次大输出直接让机械臂
    发生“瞬移”或危险的机械冲击。

    Args:
        value (float): 本次 PID 计算出的期望新值（可能很大）。
        previous (float): 实际已经执行的上一次输出值。
        acceleration (float): 允许的最大物理加速度（或变化率）。
        dt (float): 时间步长 (秒)。

    Returns:
        float: 受到速率限制处理后的安全控制输出值。
    """
    maximum_delta = float(acceleration) * float(dt)
    # 计算期望变化量，并夹紧在 [-maximum_delta, +maximum_delta] 范围内
    delta = max(-maximum_delta, min(maximum_delta, float(value) - float(previous)))
    return float(previous) + delta


def limit_xy_norm(x, y, maximum):
    """
    按范数比例缩放 XY 二维向量，限制其最大长度。

    假设控制指令为一个二维向量 (x, y)，必须限制其整体长度不能超过 `maximum`。
    如果 `x` 和 `y` 独立限幅（例如都限制在 maximum 内），会导致物理量在 45度角
    方向时的实际长度超出 1.414 倍。

    此处使用 `math.hypot` 计算欧氏范数，并等比例缩放 X 和 Y 分量，
    保证了整体速度/角速度方向不会因为限幅而改变。

    Args:
        x (float): X 轴控制分量。
        y (float): Y 轴控制分量。
        maximum (float): 允许的最大向量欧氏范数。

    Returns:
        tuple: (scaled_x, scaled_y) 受限幅处理后的分量。
    """
    norm = math.hypot(x, y)
    if norm > maximum and norm > 1e-9:
        scale = maximum / norm
        return float(x) * scale, float(y) * scale
    return float(x), float(y)


class SimpleTargetPredictor2D:
    """
    二维视觉目标运动预测器（简易航位推算）。

    视觉感知（YOLO 推理）有一小部分固有延迟。为了补偿这个延迟，
    此预测器记录目标最后的位置和速度，并根据设定的小量前馈时间
    (`predict_lead_sec`)，推算出目标在“未来一小段时间后”的期望位置。
    这能使 PID 控制器更平滑地跟踪移动的目标。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        """清空所有追踪状态。"""
        self._position = (0.0, 0.0)
        self._velocity = (0.0, 0.0)
        self._stamp = None  # 最后一次更新的时间戳

    def update(self, position, velocity, stamp):
        """
        更新目标在当前时刻的位置和速度快照。

        Args:
            position (tuple): (u, v) 像素坐标下的目标位置。
            velocity (tuple): (du/dt, dv/dt) 像素坐标下的目标移动速度。
            stamp (float): 当前数据的绝对时间戳 (秒)。
        """
        self._position = (float(position[0]), float(position[1]))
        self._velocity = (float(velocity[0]), float(velocity[1]))
        self._stamp = float(stamp)

    def predict_to(self, stamp, max_horizon):
        """
        预测目标在指定未来时间点的位置。

        核心逻辑：使用简单的线性外推公式 `位置 = 当前位置 + 速度 * 时间差`。

        Args:
            stamp (float): 期望预测的未来时间戳。
            max_horizon (float): 最大前馈预测时间限制，防止时间差无限增大。

        Returns:
            tuple: (predicted_position, current_velocity)
                如果未初始化或时间倒流，则返回 (None, None)。
        """
        if self._stamp is None:
            return None, None
        # 计算时间差，并限制在最大预测时间范围内
        dt = max(0.0, min(float(max_horizon), float(stamp) - self._stamp))
        return (
            tuple(position + velocity * dt
                  for position, velocity in zip(self._position, self._velocity)),
            self._velocity,
        )
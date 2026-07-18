from dataclasses import dataclass


@dataclass(frozen=True)
class ServoControlConfig:
    kp_xy: float = 0.25
    ki_xy: float = 0.0
    kd_xy: float = 0.01
    d_ema_alpha: float = 0.65
    derivative_clip_xy: float = 2.0
    integral_limit_xy: float = 0.10


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class PIDController2D:
    """Two-axis PID for RGB image-based visual servoing."""

    def __init__(self, config):
        self.cfg = config
        self.reset()

    def reset(self):
        self._last = None
        self._derivative = (0.0, 0.0)
        self._integral = (0.0, 0.0)

    def step(self, error, dt):
        dt = max(1e-3, float(dt))
        error = (float(error[0]), float(error[1]))
        if self._last is None:
            raw = (0.0, 0.0)
        else:
            raw = tuple(
                _clamp((value - previous) / dt, self.cfg.derivative_clip_xy)
                for value, previous in zip(error, self._last)
            )
        alpha = float(self.cfg.d_ema_alpha)
        derivative = tuple(
            alpha * value + (1.0 - alpha) * previous
            for value, previous in zip(raw, self._derivative)
        )
        integral = tuple(
            _clamp(total + value * dt, self.cfg.integral_limit_xy)
            for total, value in zip(self._integral, error)
        )
        command = tuple(
            self.cfg.kp_xy * value
            + self.cfg.ki_xy * total
            + self.cfg.kd_xy * rate
            for value, total, rate in zip(error, integral, derivative)
        )
        self._last = error
        self._derivative = derivative
        self._integral = integral
        return command[0], command[1], {
            'error': error,
            'derivative': derivative,
            'integral': integral,
        }

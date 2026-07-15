from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ServoControlConfig:
    kp_xy: float = 0.25
    ki_xy: float = 0.0
    kd_xy: float = 0.01
    d_ema_alpha: float = 0.65
    derivative_clip_xy: float = 2.0
    integral_limit_xy: float = 0.10


class PIDController3D:
    """PID interface retained from Fairino; Z is disabled for RGB-only IBVS."""

    def __init__(self, config):
        self.cfg = config
        self.reset()

    def reset(self):
        self._last = None
        self._derivative = np.zeros(3, dtype=float)
        self._integral = np.zeros(3, dtype=float)

    def step(self, error, dt):
        dt = float(np.clip(dt, 1e-3, 5e-2))
        error = np.asarray(error, dtype=float).reshape(3,)
        raw = (
            np.zeros(3, dtype=float)
            if self._last is None else (error - self._last) / dt)
        raw[:2] = np.clip(
            raw[:2], -self.cfg.derivative_clip_xy,
            self.cfg.derivative_clip_xy)
        alpha = float(self.cfg.d_ema_alpha)
        derivative = alpha * raw + (1.0 - alpha) * self._derivative
        self._integral[:2] += error[:2] * dt
        self._integral[:2] = np.clip(
            self._integral[:2], -self.cfg.integral_limit_xy,
            self.cfg.integral_limit_xy)
        command = (
            self.cfg.kp_xy * error
            + self.cfg.ki_xy * self._integral
            + self.cfg.kd_xy * derivative)
        command[2] = 0.0
        self._last = error.copy()
        self._derivative = derivative.copy()
        return float(command[0]), float(command[1]), 0.0, {
            'error': error.copy(),
            'derivative': derivative.copy(),
            'integral': self._integral.copy(),
        }

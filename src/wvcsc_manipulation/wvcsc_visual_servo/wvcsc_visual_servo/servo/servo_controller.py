"""Pure angular image-based visual-servo command synthesis."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ServoControllerConfig:
    """Fixed control-law and command-shaping parameters for one Servo node."""

    control_rate_hz: float
    kp_xy: float
    kd_xy: float
    d_ema_alpha: float
    derivative_clip_xy: float
    max_angular_speed: float
    max_angular_acceleration: float
    angular_u_sign: float
    angular_v_sign: float


class ServoController:
    """Own PD smoothing, command limits, slew history and axis mapping."""

    def __init__(self, config):
        self._config = config
        self.reset()

    @property
    def last_command(self):
        return self._last_command

    def reset(self):
        self._last_error = None
        self._derivative = (0.0, 0.0)
        self._last_command = (0.0, 0.0)

    def hold_zero(self):
        """Stop output slew without discarding the PD observation history."""
        self._last_command = (0.0, 0.0)

    def command(self, error, elapsed_sec):
        """Return a bounded angular XY command for normalized image error."""
        dt = self._bounded_dt(elapsed_sec)
        error = (float(error[0]), float(error[1]))
        raw_derivative = (
            (0.0, 0.0) if self._last_error is None else tuple(
                self._clamp((value - previous) / dt,
                            self._config.derivative_clip_xy)
                for value, previous in zip(error, self._last_error)
            )
        )
        alpha = self._config.d_ema_alpha
        self._derivative = tuple(
            alpha * value + (1.0 - alpha) * previous
            for value, previous in zip(raw_derivative, self._derivative)
        )
        requested = tuple(
            self._config.kp_xy * value + self._config.kd_xy * rate
            for value, rate in zip(error, self._derivative)
        )
        requested = self._limit_norm(*requested)
        self._last_error = error
        self._last_command = tuple(
            self._slew(value, previous, dt)
            for value, previous in zip(requested, self._last_command)
        )
        return self._last_command

    def twist_components(self, command):
        """Map image-space angular command into camera-optical Twist fields."""
        x, y = (float(value) for value in command)
        return (
            0.0,
            0.0,
            -self._config.angular_v_sign * y,
            self._config.angular_u_sign * x,
        )

    def _bounded_dt(self, elapsed_sec):
        period = 1.0 / self._config.control_rate_hz
        return max(1e-3, min(2.0 * period, max(0.0, float(elapsed_sec))))

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _limit_norm(self, x, y):
        norm = math.hypot(x, y)
        if norm > self._config.max_angular_speed and norm > 1e-9:
            scale = self._config.max_angular_speed / norm
            return x * scale, y * scale
        return x, y

    def _slew(self, value, previous, dt):
        maximum_delta = self._config.max_angular_acceleration * dt
        delta = max(-maximum_delta, min(maximum_delta, value - previous))
        return previous + delta

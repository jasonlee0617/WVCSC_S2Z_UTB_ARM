"""Pure numerical helpers shared by the IBVS control loop."""
import math


def bounded_control_dt(elapsed, control_rate_hz):
    period = 1.0 / float(control_rate_hz)
    return max(1e-3, min(2.0 * period, float(elapsed)))


def slew(value, previous, acceleration, dt):
    maximum_delta = float(acceleration) * float(dt)
    delta = max(-maximum_delta, min(maximum_delta, float(value) - float(previous)))
    return float(previous) + delta


def limit_xy_norm(x, y, maximum):
    norm = math.hypot(x, y)
    if norm > maximum and norm > 1e-9:
        scale = maximum / norm
        return float(x) * scale, float(y) * scale
    return float(x), float(y)


class SimpleTargetPredictor2D:
    """Dead-reckoning predictor for 2D visual target motion."""

    def __init__(self):
        self.reset()

    def reset(self):
        self._position = (0.0, 0.0)
        self._velocity = (0.0, 0.0)
        self._stamp = None

    def update(self, position, velocity, stamp):
        self._position = (float(position[0]), float(position[1]))
        self._velocity = (float(velocity[0]), float(velocity[1]))
        self._stamp = float(stamp)

    def predict_to(self, stamp, max_horizon):
        if self._stamp is None:
            return None, None
        dt = max(0.0, min(float(max_horizon), float(stamp) - self._stamp))
        return (
            tuple(position + velocity * dt
                  for position, velocity in zip(self._position, self._velocity)),
            self._velocity,
        )

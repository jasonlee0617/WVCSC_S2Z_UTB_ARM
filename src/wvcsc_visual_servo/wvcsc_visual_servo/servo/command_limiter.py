import math


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

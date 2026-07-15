import numpy as np


class SimpleTargetPredictor2D:
    def __init__(self):
        self.reset()

    def reset(self):
        self._position = np.zeros(2, dtype=float)
        self._velocity = np.zeros(2, dtype=float)
        self._stamp = None

    def update(self, position, velocity, stamp):
        self._position = np.asarray(position, dtype=float).reshape(2,)
        self._velocity = np.asarray(velocity, dtype=float).reshape(2,)
        self._stamp = float(stamp)

    def predict_to(self, stamp, max_horizon):
        if self._stamp is None:
            return None, None
        dt = float(np.clip(float(stamp) - self._stamp, 0.0, max_horizon))
        return self._position + self._velocity * dt, self._velocity.copy()

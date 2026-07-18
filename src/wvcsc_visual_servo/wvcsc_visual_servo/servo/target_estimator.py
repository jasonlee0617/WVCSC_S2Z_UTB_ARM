class SimpleTargetPredictor2D:
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
            tuple(position + velocity * dt for position, velocity in zip(
                self._position, self._velocity)),
            self._velocity,
        )

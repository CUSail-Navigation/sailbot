import numpy as np


class FakeLicorAnemometer:
    def __init__(self):
        self.direction = float(np.random.uniform(0, 360))
        self.speed = float(np.random.uniform(0, 5))

    def read_speed(self):
        self.speed = max(0.0, self.speed + np.random.uniform(-0.5, 0.5))
        return self.speed

    def read_direction(self):
        self.direction = (self.direction + np.random.uniform(-5, 5)) % 360
        return int(self.direction)

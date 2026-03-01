import numpy as np

class Target:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
    def update(self, dt):
        self.pos += self.vel * dt

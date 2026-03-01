import numpy as np

class Motor:
    def __init__(self):
        self.lut_time = np.array([0.0, 1.0, 2.0, 3.0])
        self.lut_thrust = np.array([30.0, 50.0, 10.0, 0.0])

    def get_thrust_body(self, time_elapsed):
        thrust = np.interp(time_elapsed, self.lut_time, self.lut_thrust)
        return np.array([thrust, 0, 0])

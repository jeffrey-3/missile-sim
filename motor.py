import numpy as np

class Motor:
    def __init__(self):
        # Estes E16 Thrust Curve
        self.lut_time = np.array([0.0, 0.2, 0.5, 1.0, 2.0, 2.1])
        self.lut_thrust = np.array([0.0, 5.0, 26.0, 16.0, 16.0, 0.0])

    def get_thrust_body(self, time_elapsed):
        thrust = np.interp(time_elapsed, self.lut_time, self.lut_thrust)
        return np.array([thrust, 0, 0])

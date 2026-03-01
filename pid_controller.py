import numpy as np
from controller import Controller

class PIDController(Controller):
    def __init__(self):
        self.kP = 0.5

    def update(self, missile, target):
        to_target_world = target.pos - missile.pos
        to_target_body = missile.world_to_body(to_target_world)
        pitch_error = np.arctan2(-to_target_body[2], to_target_body[0])
        yaw_error = np.arctan2(to_target_body[1], to_target_body[0])
        u1 = pitch_error * self.kP
        u2 = yaw_error * self.kP
        return u1, u2

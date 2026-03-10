import numpy as np

class PIDController():
    def __init__(self):
        self.kP = 0.5
        self.kI = 0.0
        self.kD = 0.0
        self.prev_pitch_err = 0
        self.prev_yaw_err = 0
        self.pitch_integral = 0
        self.yaw_integral = 0

    def update(self, missile, target, dt):
        to_target_world = target.pos - missile.pos
        to_target_body = missile.world_to_body(to_target_world)

        pitch_error = np.arctan2(-to_target_body[2], to_target_body[0])
        yaw_error = np.arctan2(to_target_body[1], to_target_body[0])

        self.pitch_integral += pitch_error * dt
        self.yaw_integral += yaw_error * dt

        pitch_derivative = (pitch_error - self.prev_pitch_err) / dt
        yaw_derivative = (yaw_error - self.prev_yaw_err) / dt

        u1 = (pitch_error * self.kP + self.pitch_integral * self.kI +
            pitch_derivative * self.kD)
        u2 = (yaw_error * self.kP + self.yaw_integral * self.kI +
            yaw_derivative * self.kD)

        self.prev_pitch_err = pitch_error
        self.prev_yaw_err = yaw_error

        return u1, u2

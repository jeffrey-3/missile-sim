import numpy as np
from scipy.spatial.transform import Rotation
from surface import Surface
import numpy as np
from scipy.spatial.transform import Rotation
from surface import Surface

class Missile:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.rot = Rotation.identity()
        self.omega = np.zeros(3)
        self.mass = 0.3
        self.inertia_tensor = np.diag(np.array([0.01, 0.05, 0.05]))
        self.fin_y = Surface(
            5.0,
            0.05,
            0.03,
            np.array([-0.1, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([0, 0, 1]),
            0.05
        )
        self.fin_z = Surface(
            5.0,
            0.05,
            0.03,
            np.array([-0.1, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([0, 1, 0]),
            0.05
        )
        self.canard_y = Surface(
            5.0,
            0.0,
            0.01,
            np.array([0.05, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([0, 0, 1]),
            0.01
        )
        self.canard_z = Surface(
            5.0,
            0.0,
            0.01,
            np.array([0.05, 0, 0]),
            np.array([-1, 0, 0]),
            np.array([0, 1, 0]),
            0.01
        )
        self.motor_lut_time = np.array([0.0, 1.0, 2.0, 3.0])
        self.motor_lut_thrust = np.array([30.0, 50.0, 10.0, 0.0])

    def update(self, canard_y_angle, canard_z_angle, dt, time_elapsed):
        self.set_canard_angles(canard_y_angle, canard_z_angle)
        aero_force, moment = self.compute_force_moment()
        force_body = aero_force + self.get_thrust_body(time_elapsed)
        force_world = self.body_to_world(force_body)
        gravity_world = np.array([0, 0, self.mass * 9.81])
        linear_accel = (force_world + gravity_world) / self.mass
        self.vel += linear_accel * dt
        self.pos += self.vel * dt
        angular_accel = np.matmul(np.linalg.inv(self.inertia_tensor), moment)
        self.omega += angular_accel * dt
        delta_rot = Rotation.from_rotvec(self.omega * dt)
        self.rot = self.rot * delta_rot

    def set_canard_angles(self, canard_y_angle, canard_z_angle):
        y_canard_normal = np.array([0, 0, 1])
        if abs(canard_y_angle) > 1e-8:
            rot_y = Rotation.from_rotvec([0, canard_y_angle, 0])
            y_canard_normal = rot_y.apply(y_canard_normal)
        self.canard_y.fin_normal = self.normalize(y_canard_normal)
        z_canard_normal = np.array([0, 1, 0])
        if abs(canard_z_angle) > 1e-8:
            rot_z = Rotation.from_rotvec([0, 0, canard_z_angle])
            z_canard_normal = rot_z.apply(z_canard_normal)
        self.canard_z.fin_normal = self.normalize(z_canard_normal)

    def get_thrust_body(self, time_elapsed):
        thrust = np.interp(
            time_elapsed,
            self.motor_lut_time,
            self.motor_lut_thrust
        )
        return np.array([thrust, 0, 0])

    def set_euler(self, euler):
        self.rot = Rotation.from_euler('xyz', euler)

    def get_euler(self):
        return self.rot.as_euler('xyz')

    def body_to_world(self, v):
        return self.rot.apply(v)

    def world_to_body(self, v):
        return self.rot.inv().apply(v)

    def compute_force_moment(self):
        force_fin_y, moment_fin_y = self.fin_y.compute_force_moment(
            self.world_to_body(self.vel),
            self.omega
        )
        force_fin_z, moment_fin_z = self.fin_z.compute_force_moment(
            self.world_to_body(self.vel),
            self.omega
        )
        force_canard_y, moment_canard_y = self.canard_y.compute_force_moment(
            self.world_to_body(self.vel),
            self.omega
        )
        force_canard_z, moment_canard_z = self.canard_z.compute_force_moment(
            self.world_to_body(self.vel),
            self.omega
        )
        return (
            force_fin_y + force_fin_z + force_canard_y + force_canard_z,
            moment_fin_y + moment_fin_z + moment_canard_y + moment_canard_z
        )

    def normalize(self, v):
        n = np.linalg.norm(v)
        if n < 1e-8:
            return np.zeros_like(v)
        return v / n

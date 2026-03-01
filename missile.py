import numpy as np
from scipy.spatial.transform import Rotation
from surface import Surface
import numpy as np
from scipy.spatial.transform import Rotation
from surface import Surface
from motor import Motor

class Missile:
    def __init__(self):
        # Pose
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.accel_world = np.zeros(3)
        self.rot = Rotation.identity()
        self.omega = np.zeros(3)
        self.accel_imu = np.zeros(3)

        # Properties
        self.mass = 0.3
        self.inertia_tensor = np.diag(np.array([0.0000408, 0.00226, 0.00226]))

        # Environment
        self.gravity_world = np.array([0, 0, self.mass * 9.81])

        # Fins
        self.fin_y = Surface(5.0, 0.0, 0.1, np.array([-0.12, 0, 0]),
            np.array([-1, 0, 0]), np.array([0, 0, 1]), 0.0018)
        self.fin_z = Surface(5.0, 0.0, 0.1, np.array([-0.12, 0, 0]),
            np.array([-1, 0, 0]), np.array([0, 1, 0]), 0.0018)
        self.canard_y = Surface(5.0, 0.0, 0.1, np.array([0.14, 0, 0]),
            np.array([-1, 0, 0]), np.array([0, 0, 1]), 0.0008)
        self.canard_z = Surface(5.0, 0.0, 0.1, np.array([0.14, 0, 0]),
            np.array([-1, 0, 0]), np.array([0, 1, 0]), 0.0008)

        # Motor
        self.motor = Motor()

    def update(self, canard_y_angle, canard_z_angle, dt, time_elapsed):
        # Actuator limits
        canard_y_angle = np.clip(canard_y_angle, -np.deg2rad(10.0),
            np.deg2rad(10.0))
        canard_z_angle = np.clip(canard_z_angle, -np.deg2rad(10.0),
            np.deg2rad(10.0))

        # Set canard deflection angles
        self.set_canard_angles(canard_y_angle, canard_z_angle)

        # Get aerodynamic forces
        aero_force_body, moment = self.compute_force_moment()

        # Add motor thrust
        aero_force_body += self.motor.get_thrust_body(time_elapsed)

        # Rotate force to world frame
        force_world = self.body_to_world(aero_force_body)

        # Add gravity
        force_world += self.gravity_world

        # Calculate linear acceleration
        self.accel_world = force_world / self.mass

        # Calculate angular acceleration from aerodynamic moments
        angular_accel = np.matmul(np.linalg.inv(self.inertia_tensor), moment)

        # Propagate kinematics
        self.vel += self.accel_world * dt
        self.pos += self.vel * dt
        self.omega += angular_accel * dt
        self.rot *= Rotation.from_rotvec(self.omega * dt)

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

    def compute_force_moment(self):
        force_fin_y, moment_fin_y = self.fin_y.compute_force_moment(
            self.world_to_body(self.vel), self.omega)
        force_fin_z, moment_fin_z = self.fin_z.compute_force_moment(
            self.world_to_body(self.vel), self.omega)
        force_canard_y, moment_canard_y = self.canard_y.compute_force_moment(
            self.world_to_body(self.vel), self.omega)
        force_canard_z, moment_canard_z = self.canard_z.compute_force_moment(
            self.world_to_body(self.vel), self.omega)
        return (force_fin_y + force_fin_z + force_canard_y + force_canard_z,
            moment_fin_y + moment_fin_z + moment_canard_y + moment_canard_z)

    def set_euler(self, euler):
        self.rot = Rotation.from_euler('xyz', euler)

    def get_euler(self):
        return self.rot.as_euler('xyz')

    def body_to_world(self, v):
        return self.rot.apply(v)

    def world_to_body(self, v):
        return self.rot.inv().apply(v)

    def normalize(self, v):
        n = np.linalg.norm(v)
        if n < 1e-8:
            return np.zeros_like(v)
        return v / n

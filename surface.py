import numpy as np
from scipy.spatial.transform import Rotation

class Surface:
    # fin_chord points downstream and fin_normal is lift direction at zero AOA
    def __init__(self, Cla, Cda, Cd0, r_fin_body, fin_chord, fin_normal, S):
        self.Cla = Cla
        self.Cda = Cda
        self.Cd0 = Cd0
        self.r_fin_body = r_fin_body
        self.fin_chord = self.normalize(fin_chord)
        self.fin_normal = self.normalize(fin_normal)
        self.S = S
        self.rho = 1.225

    def compute_force_moment(self, v_body, omega_body):
        v_fin = v_body + np.cross(omega_body, self.r_fin_body)
        v_rel = -v_fin
        v_hat = self.normalize(v_rel)
        alpha = np.arctan2(np.dot(v_hat, self.fin_normal),
            np.dot(v_hat, self.fin_chord))
        drag_dir = v_hat
        lift_dir = self.normalize(np.cross(np.cross(v_hat, self.fin_normal),
            v_hat))
        q = 0.5 * self.rho * np.linalg.norm(v_rel) ** 2
        Cl = self.Cla * alpha
        Cd = self.Cda * alpha + self.Cd0
        F_drag = q * self.S * Cd * drag_dir
        F_lift = q * self.S * Cl * lift_dir
        F_total = F_lift + F_drag
        M = np.cross(self.r_fin_body, F_total)
        return F_total, M

    def normalize(self, v):
        n = np.linalg.norm(v)
        if n < 1e-8:
            return np.zeros_like(v)
        return v / n

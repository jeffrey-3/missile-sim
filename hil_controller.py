import numpy as np
import serial
from controller import Controller

class HILController(Controller):
    def __init__(self, port: str, baudrate: int):
        self.ser = serial.Serial(port, baudrate)
        self.servo_y_angle = 0.0
        self.servo_z_angle = 0.0

    def update(self, missile, target):
        s = (f"{missile.omega[0]:.3f},{missile.omega[1]:.3f},"
             f"{missile.omega[2]:.3f},{missile.accel_imu[0]:.3f},"
             f"{missile.accel_imu[1]:.3f},{missile.accel_imu[2]:.3f}\r")
        self.ser.write(s.encode('utf-8'))

        line = self.ser.readline()
        try:
            line = line.decode("utf-8").strip().split(",")
            line = [float(v) for v in line]
            print(line)

            max_pulse = 2100.0
            min_pulse = 900.0
            middle_pulse = (max_pulse + min_pulse) / 2.0
            pulse_range = max_pulse - min_pulse
            deflection_range = np.rad(10.0)
            line[0] = np.clamp(line[0], min_pulse, max_pulse)
            line[1] = np.clamp(line[1], min_pulse, max_pulse)
            self.servo_y_angle = (line[0] - middle_pulse) / pulse_range
                * deflection_range
            self.servo_z_angle = (line[1] - middle_pulse) / pulse_range
                * deflection_range
        except ValueError:
            print("Parse error:", line)

        return self.servo_y_angle, self.servo_z_angle

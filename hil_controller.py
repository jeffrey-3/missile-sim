import numpy as np
import serial
from controller import Controller

class HILController(Controller):
    def __init__(self, port: str, baudrate: int):
        self.ser = serial.Serial(port, baudrate)

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
        except ValueError:
            print("Parse error:", line)

        return 0, 0

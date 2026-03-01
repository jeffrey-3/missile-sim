import numpy as np
import serial
from controller import Controller

class HILController(Controller):
    def __init__(self, port: str, baudrate: int):
        self.ser = serial.Serial(port, baudrate)

    def update(self, missile, target):
        return 0, 0

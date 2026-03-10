import numpy as np
import queue
import threading
import time
from src.missile import Missile
from src.target import Target
from src.pid import PIDController
from src.plot import Plot

missile = Missile()
missile.set_euler(np.array([np.radians(0.0), np.radians(80.0),
    np.radians(0.0)]))
missile.omega[0] = np.radians(0.0)

target = Target()
target.pos = np.array([50.0, -50.0, -150.0])
target.vel = np.array([0, 0, 0])

pid = PIDController()

def simulator(queue):
    start_time = time.perf_counter()
    dt = 0.01
    for i in range(0, 1000):
        sim_time = i * dt

        while time.perf_counter() - start_time < sim_time:
            time.sleep(0.00001)

        target.update(dt)
        canard_y_angle, canard_z_angle = pid.update(missile, target, dt)
        missile.set_canard_angles(canard_y_angle, canard_z_angle)
        missile.update(dt, sim_time)

        queue.put((missile.pos, missile.rot, target.pos))

queue = queue.Queue()

thread = threading.Thread(target=simulator, args=(queue,), daemon=True)
thread.start()

plot = Plot(queue, 100)
plot.start()

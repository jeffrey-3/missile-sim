import numpy as np
import queue
import threading
import time
from missile import Missile
from target import Target
from controller import Controller
from plot import Plot

missile = Missile()
missile.set_euler(np.array([np.radians(0.0), np.radians(90.0),
    np.radians(0.0)]))

target = Target()
target.pos = np.array([-150.0, -50.0, -200.0])
target.vel = np.array([50, 0, 0])

controller = Controller()

def simulator(queue):
    dt = 0.01
    start_time = time.perf_counter()
    last_time = time.perf_counter()
    while time.perf_counter() - start_time < 100:
        current_time = time.perf_counter()
        elapsed_time = current_time - last_time

        if elapsed_time < dt:
            time.sleep(dt - elapsed_time)

        target.update(dt)
        u1, u2 = controller.update(missile, target)
        missile.update(u1, u2, dt, time.perf_counter() - start_time)

        queue.put((missile.pos, missile.rot, target.pos))

        last_time = current_time

queue = queue.Queue()

plot = Plot(queue, 150)

thread = threading.Thread(target=simulator, args=(queue,), daemon=True)
thread.start()

plot.start()

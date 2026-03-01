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
target.vel = np.array([0, 0, 0])

controller = Controller()

def simulator(queue):
    dt = 0.005
    iteration = 0
    start_time = time.perf_counter()
    while True:
        if time.perf_counter() < start_time + iteration * dt:
            time.sleep(0.0001)
            continue

        iteration += 1
        target.update(dt)
        u1, u2 = controller.update(missile, target)
        missile.update(u1, u2, dt, iteration * dt)
        queue.put((missile.pos, missile.rot, target.pos))

queue = queue.Queue()

thread = threading.Thread(target=simulator, args=(queue,), daemon=True)
thread.start()

plot = Plot(queue, 150)
plot.start()

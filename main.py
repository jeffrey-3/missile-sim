import numpy as np
import queue
import threading
import time
from missile import Missile
from target import Target
from plot import Plot
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--pid", action="store_true", help="Use PID controller")
parser.add_argument("--hil", action="store_true", help="Enable HIL testing")
args = parser.parse_args()

if args.pid:
    from pid_controller import PIDController
    controller = PIDController()
elif args.hil:
    from hil_controller import HILController
    controller = HILController("/dev/ttyACM0", 115200)
else:
    from pid_controller import PIDController
    controller = PIDController()

missile = Missile()
missile.set_euler(np.array([np.radians(0.0), np.radians(90.0),
    np.radians(0.0)]))

target = Target()
target.pos = np.array([-100.0, -50.0, -150.0])
target.vel = np.array([0, 0, 0])

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

plot = Plot(queue, 100)
plot.start()

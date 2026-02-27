import numpy as np
import queue
import threading
import time
from missile import Missile
from plot import Plot

missile = Missile()
missile.set_euler(
    np.array([np.radians(0.0), np.radians(90.0), np.radians(0.0)])
)

def simulator(queue):
    target_pos = np.array([-150.0, -50.0, -200.0])

    # Show initial position for a second before starting
    for i in range(10):
        queue.put((missile.pos, missile.rot, target_pos))
    time.sleep(1)

    dt = 0.01
    start_time = time.perf_counter()
    last_time = time.perf_counter()
    while time.perf_counter() - start_time < 100:
        current_time = time.perf_counter()
        elapsed_time = current_time - last_time

        if elapsed_time < dt:
            time.sleep(dt - elapsed_time)

        # Update target
        target_pos[0] += 80 * dt

        # Find error to target
        to_target_world = target_pos - missile.pos
        to_target_body = missile.world_to_body(to_target_world)
        pitch_error = np.arctan2(-to_target_body[2], to_target_body[0])
        yaw_error = np.arctan2(to_target_body[1], to_target_body[0])

        # P controller
        u1 = pitch_error * 0.1
        u2 = yaw_error * 0.1

        missile.update(u1, u2, dt, time.perf_counter() - start_time)

        queue.put((missile.pos, missile.rot, target_pos))

        last_time = current_time

queue = queue.Queue()

plot = Plot(queue, 150)

thread = threading.Thread(target=simulator, args=(queue,), daemon=True)
thread.start()

plot.start()

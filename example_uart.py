import numpy as np
import queue
import threading
import serial
from src.missile import Missile
from src.target import Target
from src.plot import Plot

ser = serial.Serial("/dev/ttyACM0", 1000000)

missile = Missile()
missile.set_euler(np.array([np.radians(0.0), np.radians(80.0),
    np.radians(0.0)]))
missile.omega[0] = np.radians(0.0)

target = Target()
target.pos = np.array([50.0, -50.0, -150.0])
target.vel = np.array([0, 0, 0])

def simulator(queue):
    start_time = None
    while True:
        line = ser.readline()
        try:
            # Parse time and actuator information
            arr = line.decode("utf-8").strip().split(",")
            time = float(arr[0]) / 1000 # Convert ms to s
            if start_time == None:
                start_time = time
            time -= start_time
            dt = float(arr[1]) / 1000
            canard_y_pulse = int(arr[2])
            canard_z_pulse = int(arr[3])
        except ValueError:
            print("Parse error:", line)
            break
        else:
            # Update simulator
            target.update(dt)
            missile.set_canard_pulse(canard_y_pulse, canard_z_pulse)
            missile.update(dt, time)

            # Send updated sensors to missile
            s = (f"{missile.omega[0]:.5f},{missile.omega[1]:.5f},"
                 f"{missile.omega[2]:.5f},{missile.accel_imu[0]:.5f},"
                 f"{missile.accel_imu[1]:.5f},{missile.accel_imu[2]:.5f}\r")
            s = s.encode('utf-8')
            ser.write(s)

            # Add to plot queue
            queue.put((missile.pos, missile.rot, target.pos))


queue = queue.Queue()

thread = threading.Thread(target=simulator, args=(queue,), daemon=True)
thread.start()

plot = Plot(queue, 100)
plot.start()

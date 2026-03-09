import numpy as np
import queue
import threading
import serial
from missile import Missile
from target import Target
from plot import Plot

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

    missile.update_accel_imu()

    while True:
        # Wait for update request and get time information
        line = ser.readline()
        try:
            arr = line.decode("utf-8").strip().split(",")
            time = float(arr[0]) / 1000 # Convert ms to s
            if start_time == None:
                start_time = time
            time -= start_time
            dt = float(arr[1]) / 1000
            print(time, dt)

            max_pulse = 2100.0
            min_pulse = 900.0
            middle_pulse = (max_pulse + min_pulse) / 2.0
            pulse_range = max_pulse - min_pulse
            deflection_range = np.deg2rad(10.0)
            u1 = np.clip(int(arr[2]), min_pulse, max_pulse)
            u2 = np.clip(int(arr[3]), min_pulse, max_pulse)
            u1 = (u1 - middle_pulse) / (pulse_range * deflection_range)
            u2 = (u2 - middle_pulse) / (pulse_range * deflection_range)
        except ValueError:
            print("Parse error:", line)

        # Update simulator
        target.update(dt)
        missile.update(u1, u2, dt, time)
        queue.put((missile.pos, missile.rot, target.pos))

        # Send updated sensors to missile
        s = (f"{missile.omega[0]:.5f},{missile.omega[1]:.5f},"
             f"{missile.omega[2]:.5f},{missile.accel_imu[0]:.5f},"
             f"{missile.accel_imu[1]:.5f},{missile.accel_imu[2]:.5f}\r")
        s = s.encode('utf-8')
        ser.write(s)

queue = queue.Queue()

thread = threading.Thread(target=simulator, args=(queue,), daemon=True)
thread.start()

plot = Plot(queue, 100)
plot.start()

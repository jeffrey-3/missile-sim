import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import queue

class Plot:
    def __init__(self, queue, limits):
        self.queue = queue
        self.limits = limits
        self.x_data = []
        self.y_data = []
        self.z_data = []
        plt.rcParams['toolbar'] = 'None'
        self.fig = plt.figure("Simulation")
        canvas = self.fig.canvas
        self.ax = self.fig.add_subplot(111, projection='3d')
        for cid, func in list(canvas.callbacks.callbacks.get(
            'button_press_event', {}).items()):
            canvas.mpl_disconnect(cid)
        self.ax.set_xlim(-self.limits, self.limits)
        self.ax.set_ylim(-self.limits, self.limits)
        self.ax.set_zlim(-self.limits*2, 0)
        self.ax.set_box_aspect((1, 1, 1))
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.invert_yaxis()
        self.ax.invert_zaxis()
        self.ax.view_init(elev=10, azim=135)
        self.line, = self.ax.plot([], [], [], color=(0, 0, 0))
        self.x_axis, = self.ax.plot([], [], [], lw=2, color=(1, 0, 0))
        self.y_axis, = self.ax.plot([], [], [], lw=2, color=(0, 1, 0))
        self.z_axis, = self.ax.plot([], [], [], lw=2, color=(0, 0, 1))
        self.target_line, = self.ax.plot([], [], [], color=(0, 0, 0),
            linestyle='--')
        self.target, = self.ax.plot([], [], [], color=(0, 0, 0),
            linestyle='none', marker='*', markersize=10)

    def get_data(self):
        data = None
        while self.queue.qsize() > 3:
            data = self.queue.get_nowait()
        return data

    def update(self, frame):
        data = self.get_data()
        if data:
            pos = data[0]
            rot = data[1]
            target_pos = data[2]
            self.x_data.append(pos[0])
            self.y_data.append(pos[1])
            self.z_data.append(pos[2])
            rotation_matrix = rot.as_matrix()
            x_axis_new = rotation_matrix[:, 0] * self.limits / 5
            y_axis_new = rotation_matrix[:, 1] * self.limits / 5
            z_axis_new = rotation_matrix[:, 2] * self.limits / 5
            self.x_axis.set_data([pos[0], pos[0] + x_axis_new[0]],
                [pos[1], pos[1] + x_axis_new[1]])
            self.x_axis.set_3d_properties([pos[2], pos[2] + x_axis_new[2]])
            self.y_axis.set_data([pos[0], pos[0] + y_axis_new[0]],
                [pos[1], pos[1] + y_axis_new[1]])
            self.y_axis.set_3d_properties([pos[2], pos[2] + y_axis_new[2]])
            self.z_axis.set_data([pos[0], pos[0] + z_axis_new[0]],
                [pos[1], pos[1] + z_axis_new[1]])
            self.z_axis.set_3d_properties([pos[2], pos[2] + z_axis_new[2]])
            self.line.set_data(self.x_data, self.y_data)
            self.line.set_3d_properties(self.z_data)
            self.target.set_data([target_pos[0]], [target_pos[1]])
            self.target.set_3d_properties([target_pos[2]])
            self.target_line.set_data([pos[0], target_pos[0]],
                [pos[1], target_pos[1]])
            self.target_line.set_3d_properties([pos[2], target_pos[2]])
        return (self.line, self.x_axis, self.y_axis, self.z_axis,
            self.target, self.target_line)

    def start(self):
        ani = FuncAnimation(self.fig, self.update, blit=True, interval=33,
            cache_frame_data=False)
        plt.show()

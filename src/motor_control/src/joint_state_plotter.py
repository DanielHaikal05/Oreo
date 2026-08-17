#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
from time import time


class JS_Plotter(Node):
    def __init__(self):
        super().__init__('js_plotter')
        self.sub = self.create_subscription(JointState, 'joint_state', self.update_values, 10)

        self.window_size = 10
        self.t = np.array([])
        self.qd1 = np.array([])
        self.qd2 = np.array([])
        self.qd3 = np.array([])
        self.qdd1 = np.array([])
        self.qdd2 = np.array([])
        self.qdd3 = np.array([])

        plt.ion()
        self.fig1, self.ax1 = plt.subplots()

        self.v1, = self.ax1.plot([], [], label='Wheel 1')
        self.v2, = self.ax1.plot([], [], label='Wheel 2')
        self.v3, = self.ax1.plot([], [], label='Wheel 3')

        self.ax1.set_xlabel('t')
        self.ax1.set_ylabel('W')
        self.ax1.set_title('Wheel velocities in rad/s')
        self.ax1.legend()
        self.ax1.grid(True)
        plt.show(block=False)

        self.fig2, self.ax2 = plt.subplots()

        self.a1, = self.ax2.plot([], [], label='Wheel 1')
        self.a2, = self.ax2.plot([], [], label='Wheel 2')
        self.a3, = self.ax2.plot([], [], label='Wheel 3')

        self.ax2.set_xlabel('t')
        self.ax2.set_ylabel('W')
        self.ax2.set_title('Wheel accelerations in rad/s^2')
        self.ax2.legend()
        self.ax2.grid(True)
        plt.show(block=False)

        self.t0 = time()

    def update_values(self, msg):
        t = time() - self.t0
        qd = msg.velocity
        qdd = msg.effort

        self.t = np.append(self.t, t)
        keep = self.t >= t - self.window_size   

        self.t = self.t[keep]
        self.qd1 = np.append(self.qd1, qd[0])[keep]
        self.qd2 = np.append(self.qd2, qd[1])[keep]
        self.qd3 = np.append(self.qd3, qd[2])[keep]
        self.qdd1 = np.append(self.qdd1, qdd[0])[keep]
        self.qdd2 = np.append(self.qdd2, qdd[1])[keep]
        self.qdd3 = np.append(self.qdd3, qdd[2])[keep]  

        self.update_plots()

    def update_plots(self):
        self.v1.set_data(self.t, self.qd1)
        self.v2.set_data(self.t, self.qd2)
        self.v3.set_data(self.t, self.qd3)

        self.ax1.set_xlim(max(0, self.t[-1] - self.window_size), max(self.window_size, self.t[-1]))
        self.ax1.relim()
        self.ax1.autoscale_view(scalex=False, scaley=True)

        self.a1.set_data(self.t, self.qdd1)
        self.a2.set_data(self.t, self.qdd2)
        self.a3.set_data(self.t, self.qdd3)

        self.ax2.set_xlim(max(0, self.t[-1] - self.window_size), max(self.window_size, self.t[-1]))
        self.ax2.relim()
        self.ax2.autoscale_view(scalex=False, scaley=True)

        self.fig1.canvas.draw_idle()
        self.fig1.canvas.flush_events()

        self.fig2.canvas.draw_idle()
        self.fig2.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = JS_Plotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
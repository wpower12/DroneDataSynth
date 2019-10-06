from math import cos, sin
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

class Animator():
	def __init__(self):
		plt.ion()
		self.fig = plt.figure()
		self.ax  = self.fig.add_subplot(111, projection='3d')

	def plot_drones(self, drones):
		plt.cla()
		for d in drones:
			self.plot_drone(d)

		# Hardcoding for now, should fix later.
		# read the limits from the waypoints? idk.
		plt.xlim(-5, 5)
		plt.ylim(-5, 5)
		self.ax.set_zlim(0, 10)
		plt.pause(0.001)

	def plot_drone(self, d):
		T = d.transformation_matrix()
		p1_t = np.matmul(T, d.p1)
		p2_t = np.matmul(T, d.p2)
		p3_t = np.matmul(T, d.p3)
		p4_t = np.matmul(T, d.p4)

		self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]], 
			         [p1_t[1], p2_t[1], p3_t[1], p4_t[1]], 
			         [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

		self.ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
		             [p1_t[2], p2_t[2]], 'r-')
		self.ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
		             [p3_t[2], p4_t[2]], 'r-')

		self.ax.plot(d.x_data, d.y_data, d.z_data, 'b:')

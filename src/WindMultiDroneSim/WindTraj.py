# Wraps the trajectory generator so that it can accomodate a
# notion of wind. 
# Uses the wind parameters to calculate an adjusted end point
# then returns a trajectory that assumes the wind acting on
# the drone at each dt.

# Drone should STILL hit the same points in the square, regardless 
# of the wind control parameters.
from TrajectoryGenerator import *
import numpy as np

class WindTraj(TrajectoryGenerator):
	def __init__(self, start_pos, des_pos, T, start_vel=[0,0,0], des_vel=[0,0,0], start_acc=[0,0,0], des_acc=[0,0,0], wind_vel=[0,0,0]):
		super().__init__(start_pos, des_pos, T, start_vel, des_vel, start_acc, des_acc)

		# Adjust destinations for wind
		self.des_x = des_pos[0] - (wind_vel[0]*T)
		self.des_y = des_pos[1] - (wind_vel[1]*T)
		self.des_z = des_pos[2] - (wind_vel[2]*T)



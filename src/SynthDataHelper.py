import numpy as np
import math
import noise

class SynthDataHelper:
	def __init__(self):
		self.DT          = 0.5 # 'Time' steps
		# Noise Parameters
		self.NOISE_MAG   = 10.0
		self.NOISE_SCALE = 50.0
		self.NOISE_OCTS  = 1
		self.Nx_0        = 10.0
		self.Ny_0        = 100.0
		self.Nz_0        = 200.0

	def noise1(self, b, i):
		return self.NOISE_MAG*noise.pnoise1(b+i*self.DT/self.NOISE_SCALE, 
							 			    self.NOISE_OCTS)

	def gen_fp(self, start, finish):
		# The number of 'steps' is equal to the distance 
		# between the two points, divided by dt
		MAG   = np.linalg.norm(finish-start)
		DIR   = (finish-start)/MAG
		STEPS = math.floor(MAG/self.DT)

		data = np.zeros((STEPS, 3))
		for i in range(STEPS):
			data[i] = DIR*i + start
			data[i][0] += self.noise1(self.Nx_0, i)
			data[i][1] += self.noise1(self.Ny_0, i)
			data[i][2] += self.noise1(self.Nz_0, i)	

		return data


	def gen_4swarm_fp(self, finish, spacing):
		# we are assuming start = 0,0,0 for simplicity
		d = spacing*0.5
		corners = [np.array([-1.0, 0.0, 0.0]),
				   np.array([ 1.0, 0.0, 0.0]),
				   np.array([-1.0, 0.0, 0.0]),
				   np.array([ 1.0, 0.0, 0.0])]

		# Need to rotate about the y axis (2nd dim) - assume 
		# that the finish vector is on the y=0 plane
		MAG   = np.linalg.norm(finish)
		DIR   = (finish)/MAG
		STEPS = math.floor(MAG/self.DT)

		# get angle between dir and the positive x-axis
		x_pos = np.array([1,0,0])
		cos_a = np.dot(DIR, x_pos) # theyre both normed already
		angle = math.acos(cos_a)
		sin_a = math.sin(angle)

		rot   = np.array([[cos_a,    0, sin_a],
			              [0,        1, 0    ],
			              [-1*sin_a, 0, cos_a]])

		# rot   = np.array([[1,     0,          0],
		# 	              [0, cos_a, -1.0*sin_a],
		# 	              [0, sin_a,      cos_a]])

		for i in range(len(corners)):
			corners[i] = np.matmul(rot, np.transpose(corners[i]))*d*2

		# corners[2] = np.add(corners[2], np.array([0,d,0]))
		# corners[3] = np.add(corners[3], np.array([0,d,0]))

		# print(corners)

		data = []
		for c in corners:
			data_c = np.zeros((STEPS, 3))
			for i in range(STEPS):
				data_c[i] = np.add(DIR*i, c)
			data.append(data_c)

		return data 
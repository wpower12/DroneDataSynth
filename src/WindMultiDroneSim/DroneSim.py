from Quadrotor import *
from WindTraj import *
from sklearn.neural_network import MLPRegressor
from sklearn.linear_model import LinearRegression
from sklearn.tree import DecisionTreeRegressor
from sklearn.multioutput import MultiOutputRegressor

class DroneSim():
	def __init__(self, wps):
		self.train_points = [ [] for i in range(len(wps)) ]
		self.learning_mode = 'store_points'
		self.models = []
		self.x_test = []
		self.timestep = -1
		self.transition_timestep = -1
		self.i_next = -1


		self.g = 9.81
		self.m = 0.2
		self.Ixx = 1
		self.Iyy = 1
		self.Izz = 1
		self.des_yaw  = 0 # Always desire a yaw of 0
		self.wind_vel = [0,0,0]

		# Proportional coefficients	
		self.Kp_x = 1
		self.Kp_y = 1
		self.Kp_z = 1
		self.Kp_roll = 25
		self.Kp_pitch = 25
		self.Kp_yaw = 25

		# Derivative coefficients
		self.Kd_x = 10
		self.Kd_y = 10
		self.Kd_z = 1

		# Time parameters
		self.T = 5    
		self.dt = 0.1
		self.t = 0 # Time step (what step WITHIN a leg)
		self.i = 0 # Which leg of the trajectory theyre on.

		self.wps = wps
		self.N_wps = len(wps[0])
	    # Add quadrotor objects - start at first waypoint.
		self.qrs   = [ None for i in range(len(wps))]
		for q in range(len(wps)):
			q_x, q_y, q_z = wps[q][0]
			self.qrs[q] = Quadrotor(q_x, q_y, q_z)


		# Trajectories, we only store the current one.
		self.trs_x = [ [] for i in range(len(wps)) ]
		self.trs_y = [ [] for i in range(len(wps)) ]
		self.trs_z = [ [] for i in range(len(wps)) ]

		# Initial trajectory
		for w in range(len(wps)):
			wp = wps[w]
			traj = WindTraj(wp[0], wp[1], self.T, wind_vel=self.wind_vel)
			traj.solve()
			self.trs_x[w] = traj.x_c
			self.trs_y[w] = traj.y_c
			self.trs_z[w] = traj.z_c


	def tick(self):		
    	# In the future, this might need to have a t for each quad. 
    	# we are going to assume all legs of the journey are the same length across quads
			
		# If in training region....

		if self.t <= self.T:
			for d in range(len(self.qrs)):
				self.updateDrone(d)
			self.t += self.dt
		else:
			self.t = 0
			self.i = (self.i+1) % self.N_wps
			self.i_next = (self.i+1) % self.N_wps

			print('i_next:', self.i_next)

			# Calculate the new trajectories
			for w in range(len(self.qrs)):
				wp = self.wps[w]
				traj = WindTraj(wp[self.i], wp[self.i_next], self.T, wind_vel=self.wind_vel)
				traj.solve()
				self.trs_x[w] = traj.x_c
				self.trs_y[w] = traj.y_c
				self.trs_z[w] = traj.z_c
			
			'''
			if i_next == 3:
				print('Training ...')

				# construct Ytrain[d] from Xtrain[d], for each d
				for d in range(len(self.qrs)):

					Xtrain = np.zeros((len(self.train_points[d])-1,3), dtype=float)
					Ytrain = np.zeros((len(self.train_points[d])-1,3), dtype=float)
					for k in range(0,len(self.train_points[d])-1):
						Xtrain[k,:] = self.train_points[d][k]
						Ytrain[k,:] = self.train_points[d][k+1]

					# model.fit(Xtrain, Ytrain)
					curr_model = MLPRegressor()
					curr_model.fit(Xtrain,Ytrain)
					self.models.append(curr_model)

					print('d = ' + str(d) + ': done')

					self.x_test.append( np.array(self.train_points[d][-1]).reshape(1,3) )

				import time
				time.sleep(5)
				self.learning_mode = 'test'
				print('self.learning_mode = \'test\'')
			'''





	def updateDrone(self, d):
		drone = self.qrs[d]
		x_vel = drone.x_vel 
		y_vel = drone.y_vel 
		z_vel = drone.z_vel
		x_pos = drone.x
		y_pos = drone.y 
		z_pos = drone.z
		roll  = drone.roll 
		pitch = drone.pitch
		yaw   = drone.yaw  
		roll_vel = drone.roll_vel
		pitch_vel = drone.pitch_vel
		yaw_vel = drone.yaw_vel

		# Getting coefficients for current step in
		# the current trajectory
		x_c = self.trs_x[d]
		y_c = self.trs_y[d]
		z_c = self.trs_z[d]
		des_z_pos = calculate_position(z_c, self.t)
		des_z_vel = calculate_velocity(z_c, self.t)
		des_x_acc = calculate_acceleration(x_c, self.t)
		des_y_acc = calculate_acceleration(y_c, self.t)
		des_z_acc = calculate_acceleration(z_c, self.t)


		thrust = self.m*(self.g+
					     des_z_acc+
						 self.Kp_z*(des_z_pos - z_pos)+
						 self.Kd_z*(des_z_vel - z_vel))

		roll_torque  = self.Kp_roll *(((des_x_acc * sin(self.des_yaw) - des_y_acc * cos(self.des_yaw)) / self.g) - roll)
		pitch_torque = self.Kp_pitch*(((des_x_acc * cos(self.des_yaw) - des_y_acc * sin(self.des_yaw)) / self.g) - pitch)
		yaw_torque   = self.Kp_yaw  *(self.des_yaw - yaw)

		roll_vel  += roll_torque  * self.dt / self.Ixx
		pitch_vel += pitch_torque * self.dt / self.Iyy
		yaw_vel   += yaw_torque   * self.dt / self.Izz

		roll  += roll_vel  * self.dt
		pitch += pitch_vel * self.dt
		yaw   += yaw_vel   * self.dt

		R = rotation_matrix(roll, pitch, yaw)
		acc = (np.matmul(R, np.array([0, 0, thrust.item()]).T) - np.array([0, 0, self.m * self.g]).T) / self.m
		x_acc = acc[0]
		y_acc = acc[1]
		z_acc = acc[2]
		x_vel += x_acc * self.dt
		y_vel += y_acc * self.dt
		z_vel += z_acc * self.dt

		# if youre trianing - fit paramets
		x_pos += (x_vel + self.wind_vel[0]) * self.dt
		y_pos += (y_vel + self.wind_vel[1]) * self.dt
		z_pos += (z_vel + self.wind_vel[2]) * self.dt


		curr_state = np.array([x_pos, y_pos, z_pos, roll, pitch, yaw, roll_vel, pitch_vel, yaw_vel,
							   self.wps[d][self.i_next][0], self.wps[d][self.i_next][1], self.wps[d][self.i_next][2],
							   x_vel, y_vel, z_vel])

							   #self.trs_x[d].x_c, self.trs_y[d].y_c, self.trs_z[d].z_c])


		num_feats = len(curr_state)


		if self.learning_mode == 'store_points':
			self.train_points[d].append( curr_state )
		else:
			y_test_d = curr_state.reshape(1,num_feats)

			# predict the next point using the learned models
			y_pred_d = self.models[d].predict( self.x_test[d] )

			#print( 'd = ' + str(d) + ':\t' + str(y_test_d) + '\t\t' + str(y_pred_d) )

			# set y_pred_d to be the next x_test_d
			self.x_test[d] = y_pred_d

			
			x_pos = y_pred_d[0][0]
			y_pos = y_pred_d[0][1]
			z_pos = y_pred_d[0][2]
			roll = y_pred_d[0][3]
			pitch = y_pred_d[0][4]
			yaw = y_pred_d[0][5]
			roll_vel = y_pred_d[0][6]
			pitch_vel = y_pred_d[0][7]
			yaw_vel = y_pred_d[0][8]

			x_vel = y_pred_d[0][12]
			y_vel = y_pred_d[0][13]
			z_vel = y_pred_d[0][14]
			


		drone.update_pose(x_pos, y_pos, z_pos, roll, pitch, yaw)
		drone.x_vel = x_vel
		drone.y_vel = y_vel
		drone.z_vel = z_vel
		drone.roll_vel = roll_vel
		drone.pitch_vel = pitch_vel
		drone.yaw_vel = yaw_vel


		# ===========================================================================
		if self.timestep == self.transition_timestep and d == 0:
			print('Training ...')

			# construct Ytrain[d] from Xtrain[d], for each d
			for d in range(len(self.qrs)):

				Xtrain = np.zeros((len(self.train_points[d])-1,num_feats), dtype=float)
				Ytrain = np.zeros((len(self.train_points[d])-1,num_feats), dtype=float)
				for k in range(0,len(self.train_points[d])-1):
					Xtrain[k,:] = self.train_points[d][k]
					Ytrain[k,:] = self.train_points[d][k+1]

				# model.fit(Xtrain, Ytrain)
				curr_model = MultiOutputRegressor(LinearRegression())
				curr_model.fit(Xtrain,Ytrain)
				self.models.append(curr_model)

				print('d = ' + str(d) + ': done')

				self.x_test.append( np.array(self.train_points[d][-1]).reshape(1,num_feats) )

			import time
			#time.sleep(5)
			self.learning_mode = 'test'
			print('self.learning_mode = \'test\'')
		# ===========================================================================




def calculate_position(c, t):
    """
    Calculates a position given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial 
            trajectory generator.
        t: Time at which to calculate the position

    Returns
        Position
    """
    return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]


def calculate_velocity(c, t):
    """
    Calculates a velocity given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial 
            trajectory generator.
        t: Time at which to calculate the velocity

    Returns
        Velocity
    """
    return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]


def calculate_acceleration(c, t):
    """
    Calculates an acceleration given a set of quintic coefficients and a time.

    Args
        c: List of coefficients generated by a quintic polynomial 
            trajectory generator.
        t: Time at which to calculate the acceleration

    Returns
        Acceleration
    """
    return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]


def rotation_matrix(roll, pitch, yaw):
    """
    Calculates the ZYX rotation matrix.

    Args
        Roll: Angular position about the x-axis in radians.
        Pitch: Angular position about the y-axis in radians.
        Yaw: Angular position about the z-axis in radians.

    Returns
        3x3 rotation matrix as NumPy array
    """
    return np.array(
        [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
         [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
          sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
         [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
         ])

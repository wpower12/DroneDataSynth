from Quadrotor import *
from TrajectoryGenerator import *

class DroneSim():
	def __init__(self, wps):
		self.g = 9.81
		self.m = 0.2
		self.Ixx = 1
		self.Iyy = 1
		self.Izz = 1
		self.des_yaw  = 0 # Always desire a yaw of 0

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

		self.N_wps = len(wps[0])
	    # Add quadrotor objects - start at first waypoint.
		self.qrs   = [ None for i in range(len(wps))]
		for q in range(len(wps)):
			q_x, q_y, q_z = wps[q][0]
			self.qrs[q] = Quadrotor(q_x, q_y, q_z)


		# Trajectories, 
		self.trs_x = [[ [] for i in range(len(wps[0])) ],
		         [ [] for i in range(len(wps[1])) ]]
		self.trs_y = [[ [] for i in range(len(wps[0])) ],
		         [ [] for i in range(len(wps[1])) ]]
		self.trs_z = [[ [] for i in range(len(wps[0])) ],
		         [ [] for i in range(len(wps[1])) ]]

		for w in range(len(wps)):
			wp = wps[w]
			for i in range(len(wp)):
				traj = TrajectoryGenerator(wp[i], wp[(i + 1) % len(wp)], self.T)
				traj.solve()
				self.trs_x[w][i] = traj.x_c
				self.trs_y[w][i] = traj.y_c
				self.trs_z[w][i] = traj.z_c

	def tick(self):		
    	# In the future, this might need to have a t for each quad. 
    	# we are going to assume all legs of the journey are the same length across quads
		if self.t <= self.T:
			for d in range(len(self.qrs)):
				self.updateDrone(d)
			self.t += self.dt
		else:
			self.t = 0
			self.i = (self.i+1) % self.N_wps

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
		x_c = self.trs_x[d][self.i]
		y_c = self.trs_y[d][self.i]
		z_c = self.trs_z[d][self.i]
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
		x_pos += x_vel * self.dt
		y_pos += y_vel * self.dt
		z_pos += z_vel * self.dt

		drone.update_pose(x_pos, y_pos, z_pos, roll, pitch, yaw)
		drone.x_vel = x_vel
		drone.y_vel = y_vel
		drone.z_vel = z_vel
		drone.roll_vel = roll_vel
		drone.pitch_vel = pitch_vel
		drone.yaw_vel = yaw_vel

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

# Generates data for a synthetic drone flight.
# Drone will fly from (x0, y0, z0) to (xn, yn, zn)
# at an average speed s, the data will be split into
# time slices that are t wide, for a full length of 
# T = distance/speed. The data will be perturbed by 
# noise generated from a perlin noise function, of 
# scale d, such that the noise is smooth over the 
# flight path.
import numpy as np
import math
import noise
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

P_0 = np.array([0,0,0])
P_n = np.array([1200,780,500])
DT  = 0.5

# Noise Parameters
NOISE_MAG   = 10.0
NOISE_SCALE = 50.0
NOISE_OCTS  = 1
x_0 = 10.0
y_0 = 100.0
z_0 = 200.0

def noise_help(b, i):
	return noise.pnoise1(b+i*DT/NOISE_SCALE, NOISE_OCTS)*NOISE_MAG

# The number of 'steps' is equal to the distance 
# between the two points, divided by dt
MAG   = np.linalg.norm(P_n-P_0)
STEPS = math.floor(MAG/DT)

# Need the unit vector between the points to fill in the data
DIR   = (P_n-P_0)/MAG

# Now we fill in data. 
data = np.zeros((STEPS, 3))
for i in range(STEPS):
	data[i] = DIR*i
	data[i][0] += noise_help(x_0, i)
	data[i][1] += noise_help(y_0, i)
	data[i][2] += noise_help(z_0, i)

xs, ys, zs = np.split(data, 3, axis=1)

fig = plt.figure()
ax  = fig.add_subplot(111, projection='3d')

ax.plot(xs.flatten(), ys.flatten(), zs.flatten())

plt.show()
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
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from SynthDataHelper import *


P_0 = np.array([0,0,0])
P_n = np.array([1200,780,500])

gen  = SynthDataHelper()
data = gen.gen_fp(P_0, P_n) 

xs, ys, zs = np.split(data, 3, axis=1)

fig = plt.figure()
ax  = fig.add_subplot(111, projection='3d')

ax.plot(xs.flatten(), ys.flatten(), zs.flatten())

plt.show()
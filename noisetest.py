import numpy as np
import math
import noise

T = 100
dt = 0.5
N = math.floor(T/dt)

data = np.zeros((N))

for i in range(N):
	data[i] = noise.pnoise1(i*dt, 1)

print(data)
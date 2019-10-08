# Test of the translated/cleaned up drone sim code.
# creating a 'swarm' of two drones, flying in the
# same formation, just one above the other by 5 units.
from Animator import *
from DroneSim import *

# The waypoints - 'hardcoding' the swarm logic by spacing them enough.
#wps = [[[-5, -5, 0], [5, -5, 0], [5, 5, 0], [-5, 5, 0]],
#       [[-5, -5, 5], [5, -5, 5], [5, 5, 5], [-5, 5, 5]]]
wps = [[[-5, -5, 0], [-5, -1, 3], [-3, -1, 6], [5, 5, 5]]]

sim = DroneSim(wps)
anm = Animator()

sim.wind_vel = [0.01,0.01,0.01]

num_timesteps = 140
sim.transition_timestep = 120



for timestep in range(num_timesteps): # 2000
	sim.timestep = timestep
	sim.tick()
	anm.plot_drones(sim.qrs, sim.learning_mode)
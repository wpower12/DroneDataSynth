# Test of the translated/cleaned up drone sim code.
# creating a 'swarm' of two drones, flying in the
# same formation, just one above the other by 5 units.
from Animator import *
from DroneSim import *

# The waypoints - 'hardcoding' the swarm logic by spacing them enough.
wps = [[[-5, -5, 0], [5, -5, 0], [5, 5, 0], [-5, 5, 0]],
       [[-5, -5, 5], [5, -5, 5], [5, 5, 5], [-5, 5, 5]]]

sim = DroneSim(wps)
anm = Animator()

for i in range(5000):
	sim.tick()
	anm.plot_drones(sim.qrs)
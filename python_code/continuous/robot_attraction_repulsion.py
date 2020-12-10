import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from animate import *
from environments import *
from runSim import *
from funcAnimate import *



nOfRobots = 20
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
nis= [np.pi *2, np.pi * 0.2, np.pi * 0.002]
ni = nis[-1] 
#v = 0.05
v = 0.3
# Total time.

# TODO PLAY WITH THESE VALUES SEE WHAT HAPPENS TODO
obstacleRadius = 30
gridSize = 1000
T0 = 1
particle_radius = 5
torque_radius = 100 
FI0 = 1
FR0 = 1
FO0 = 1

nOfUnstuckingSteps = 600
stuckThresholdTime = 200
stuckThresholdDistance = v * 6

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 800
# Initial values of x.
x = np.zeros((1 * nOfRobots,N+1))
#x[:,0] = np.random.random(nOfRobots) * gridSize
x[:,0] = 2 * np.random.random(nOfRobots) - 1 + 500
y = np.zeros((1 * nOfRobots,N+1))
#y[:,0] = np.random.random(nOfRobots) * gridSize
y[:,0] = 2 * np.random.random(nOfRobots) - 1 + 500
#fi = np.zeros((1 * nOfRobots,N+1))
#fi[:,0] = np.random.random(nOfRobots) * 2*np.pi
robot_statesPerTime = np.zeros((1 * nOfRobots,N+1))
#x[:, 0] = 0.0


# 5 items
nOfItems = 50

percetangeOfCoverage = 0.01
delivery_station = np.array([500,500])

obstacles = initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius, delivery_station)

item_positions_set, item_positions_list = initializeItems(nOfItems, gridSize, obstacles, obstacleRadius)



x, y, nOfCollectedItemsPerTime, item_positions_listPerTime = \
    runSim(x, y, item_positions_set, delivery_station, N, nOfRobots, gridSize,  robot_statesPerTime, # sim params
                   v, particle_radius, torque_radius, obstacles,obstacleRadius,         # environment robot physical params 
                   ni, trans_dif_T, rot_dif_T,                                          # random walk params
                   T0, FR0, FI0, FO0,                                                   # artificial potential field parameters 
                   nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance)       # unstucking parameters

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
# the camera way
camera = Camera(fig)

# item_positions_list changes, you need to send a list of lists to know the changes
animate(x, y, robot_statesPerTime, item_positions_list, N, nOfRobots, particle_radius, ax, camera, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius)


animation = camera.animate()
animation.save('robot_roam=' +str(ni)  +'.mp4')

# funcanim way
#nOfSkippedFrames = 1
#anim = matplotlib.animation.FuncAnimation(fig, frameUpdate, frames=N//nOfSkippedFrames, init_func=None, fargs=(x, y, robot_statesPerTime, N, nOfRobots, particle_radius, ax, item_positions_list, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius, nOfSkippedFrames,), blit=True, cache_frame_data=True) 
#anim.save('testest.mp4')


plt.show()

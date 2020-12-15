import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from animate import *
from environments import *
from getVelocitiesFromStates import *
from realismFunctions import *

from funcAnimate import *


def onlyRandomWalks(x, y, N, nOfRobots, gridSize,  robot_statesPerTime,# sim params
                   v, particle_radius, # environment robot physical params 
                   ni, trans_dif_T, rot_dif_T,           # random walk params
                   walkType, deviation             # walktype
                   ):


    fi = np.random.random(nOfRobots) * 2*np.pi

    # 0 is search, 1 is delivering, 2 is going to pick up a spotted item, 3 if they are unstucking themselves, state 4 is levy swimming
    robot_states = np.zeros(nOfRobots)
    levySwimmers = {}

    for step in range(N):

        ####################################################################################
        # calculate movemenent stuff
        ####################################################################################

        pos = np.hstack((x[:, step].reshape((nOfRobots,1)), 
                         y[:, step].reshape((nOfRobots,1))))

        # fill this accordingly
        v_hat = np.zeros((nOfRobots, 2))
        explorers, returners, itemPickers = separateByState(robot_states, nOfRobots)

        # state 0 - active swimming (this is overwritten if it's some other state)
        if walkType == 'activeSwimming':
            activeSwimmersStyleRW(fi, ni, v_hat, explorers)


        if walkType == 'levyFlight':
            # handle state 3 - unstucking
            newLevySwimmers = generateLevyFlightSteps(v, explorers, gridSize)
            levySwimmers.update(newLevySwimmers)
            for robo in newLevySwimmers:
                robot_states[robo] = 4

            doneWithLevySwimming = set()
            #print(levySwimmers)
            for robo in levySwimmers:
                levySwimmers[robo][-1] -= 1
                if levySwimmers[robo][-1] == 0:
                    robot_states[robo] = 0
                    # can't remove dict item when iterating over the dict
                    #unstuckers.pop(robo)
                    doneWithLevySwimming.add(robo)
            for robo in doneWithLevySwimming:
                levySwimmers.pop(robo)

            levySwim(levySwimmers, v_hat)


        if walkType == 'brownianMotion':
            swimmersBrownianStyle(fi, v_hat, explorers, deviation)



        x[:, step+1] = x[:,step] +  v * v_hat[:,0]
        x[:, step+1] = x[:, step+1] % gridSize
        y[:, step+1] = y[:,step] +  v * v_hat[:,1] 
        y[:, step+1] = y[:,step+1] % gridSize

        

        robot_statesPerTime[:,step] = robot_states 
    return x, y, [], [] 





nOfRobots = 4
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
nis= [np.pi *2, np.pi * 0.2, np.pi * 0.002]
ni = nis[1] 
#v = 0.05
v = 0.3
# Total time.

# TODO PLAY WITH THESE VALUES SEE WHAT HAPPENS TODO
obstacleRadius = 30
gridSize = 1000
T0 = 0
particle_radius = 5
torque_radius = 100 
FI0 = 0
FR0 = 0
FO0 = 0

nOfUnstuckingSteps = 600
stuckThresholdTime = 200
stuckThresholdDistance = v * 6

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 5000
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
nOfItems = 0

percetangeOfCoverage = 0.0
delivery_station = np.array([500,500])

obstacles = initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius, delivery_station)

item_positions_set, item_positions_list = initializeItems(nOfItems, gridSize, obstacles, obstacleRadius)

#walkType = 'activeSwimming'
walkType = 'levyFlight'
#walkType = 'brownianMotion'

deviation = nis[1]

x, y, nOfCollectedItemsPerTime, item_positions_listPerTime = \
    onlyRandomWalks(x, y, N, nOfRobots, gridSize,  robot_statesPerTime,# sim params
                   v, particle_radius, # environment robot physical params 
                   ni, trans_dif_T, rot_dif_T, walkType, deviation)  # random walk params

fig1, ax1 = plt.subplots(1)
ax1.grid()
# camera anim
camera = Camera(fig1)
# item_positions_list changes, you need to send a list of lists to know the changes
animate(x, y, robot_statesPerTime, item_positions_list, N, nOfRobots, particle_radius, ax1, camera, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius)
animation = camera.animate()
animation.save('./vids/building_blocks/only' + walkType + '.mp4')




# funcanimation anim
#nOfSkippedFrames = 30
#item_positions = []
#anim = matplotlib.animation.FuncAnimation(fig1, frameUpdate, frames=N//nOfSkippedFrames, init_func=None, fargs=(x, y, robot_statesPerTime, N, nOfRobots, particle_radius, ax1, item_positions, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius, nOfSkippedFrames,), blit=True, cache_frame_data=True) 
#
#anim.save('testest.mp4')

# do other plots in this fashion
fig2, ax2 = plt.subplots(1)
for i in range(nOfRobots):
    ax2.set_title("walking type: " + walkType)
    ax2.plot(x[i], y[i])
plt.savefig("./prez_img/" + walkType + ".png")
plt.show()

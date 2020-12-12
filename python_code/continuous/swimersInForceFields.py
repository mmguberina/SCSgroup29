import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from scipy.stats import norm
from getForcesFromNeighbourhoods import *
from celluloid import Camera
from animate import *
from environments import *
from realismFunctions import *
from getVelocitiesFromStates import *

def swimmersInFields(x, y, item_positions_set, delivery_station, N, nOfRobots, gridSize,  robot_statesPerTime, # sim params
                   v, particle_radius, torque_radius, obstacles,obstacleRadius,         # environment robot physical params 
                   walkType, ni, trans_dif_T, rot_dif_T, deviation,                                         # random walk params
                   T0, FR0, FI0, FO0,                                                   # artificial potential field parameters 
                   nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance       # unstucking parameters
                   ):


    nOfCollectedItemsPerTime = [[0,0]]
    item_positions_list = np.array(list(item_positions_set))
    item_positions_listPerTime = [[0, item_positions_list]]
    nOfItems = len(item_positions_set)
    #fi = np.random.random(nOfRobots) * 2*np.pi
    fi = np.array([np.pi / 2])

    # 0 is search, 1 is delivering, 2 is going to pick up a spotted item, 3 if they are unstucking themselves
    # 4 is levy flight (for us that's going in one direction for some amount of time)
    robot_states = np.zeros(nOfRobots)
    robot_storage = {rob:[] for rob in range(nOfRobots)}

    levySwimmers = {}
    for step in range(N):

        ####################################################################################
        # calculate movemenent stuff
        ####################################################################################

        pos = np.hstack((x[:, step].reshape((nOfRobots,1)), 
                         y[:, step].reshape((nOfRobots,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
#        v_hat = np.hstack((np.cos(fi) .reshape((nOfRobots,1)), 
#                            np.sin(fi).reshape((nOfRobots,1))))

        # get nbhds
        robRobNeig, robItemNeig, robObsNeig = getNeighbourhoodsForTesting(pos, item_positions_list, 
                nOfRobots, nOfItems, particle_radius, torque_radius, obstacles, obstacleRadius)



        ####################################################################################
        # deal with items and states
        ####################################################################################
        # fill this accordingly
        v_hat = np.zeros((nOfRobots, 2))
        # TODO you are (correctly) calculating torques only when you're in 0 state!!!!!!!!!!!!!
        # this needs to be done so that torque calculating functions 
        # output appropriate x and y changes like everything else!
        # or have different functions depending on the state!!
        # TODO that is totally wrong!!!!!!!

        # handle all states here
        # do all calculations in separate functions
        explorers, returners, itemPickers = separateByState(robot_states, nOfRobots)


        # state 0 - active swimming (this is overwritten if it's some other state)
        # or handle state 4 - levy flight (going for some time in one direction)
        if walkType == 'activeSwimming':
            activeSwimmersStyleRW(fi, ni, v_hat, explorers)

# TODO they need to be able to switch from state 4 which they don't do right now
        if walkType == 'levyFlight':
            # handle state 3 - unstucking
            newLevySwimmers = generateLevyFlightSteps(v, explorers, gridSize)
            levySwimmers.update(newLevySwimmers)
            for robo in newLevySwimmers:
                robot_states[robo] = 4

            doneWithLevySwimming = set()
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


        # handle state 1 - delivering to station
        v_hats2DelSt = v_hat2DeliveryStationFromState(pos, delivery_station, particle_radius, robot_states, returners)
        for robo in v_hats2DelSt:
            isDone = (v_hats2DelSt[robo] == 0).all()
            # if done, drop off the item and go back to roaming
            if isDone:
                nOfCollectedItemsPerTime.append([step, nOfCollectedItemsPerTime[-1][1] + len(robot_storage[robo])])
                robot_storage[robo].clear()
                robot_states[robo] = 0
            else:
                v_hat[robo] = v_hats2DelSt[robo]



        ####################################################################################
        # superimpose forces
        ####################################################################################

        # TODO check signs, strengths and other stuff
        force_rob = calcForceRob(v, pos, robRobNeig, nOfRobots, particle_radius)
        force_rob = FR0 * force_rob
        #torque_rob = FR0 * calcTorqueRob_as_v(v, pos, robRobNeig, v_hat, nOfRobots, particle_radius)
        torque_rob = 0 * calcTorqueRob_as_v(v, pos, robRobNeig, v_hat, nOfRobots, particle_radius)

        force_item = FI0 * calcForceItem(v, pos, robItemNeig, nOfRobots, particle_radius)

        force_obs = FO0 * calcForceObs(v, pos, robObsNeig,  nOfRobots, particle_radius, obstacleRadius)

        # TODO put the coefficients in the force fuctions
        # and make them < v! (otherwise they will "discontinuously" jump)

        torque_obs = FO0  * calcTorqueObs_as_v(v, pos, robObsNeig, v_hat, nOfRobots, obstacleRadius)

        # TODO TODO TODO CHECK WHETHER THIS MAKES SENSE
        #fi = fi - torque_obs

        ####################################################################################
        # perform position updates 
        ####################################################################################

# TODO make should be chosen by some nice ifs because you will be tweaking it a lot
        x[:, step+1] = x[:,step] +  v * v_hat[:,0]
        x[:, step+1] = x[:, step+1] + force_rob[:,0] 
        x[:, step+1] = x[:, step+1] + torque_rob[:,0] 
        x[:, step+1] = x[:, step+1] - force_item[:,0] 
        x[:, step+1] = x[:, step+1] + force_obs[:,0] 
        x[:, step+1] = x[:, step+1] + torque_obs[:,0] 
        #x[:, step+1] = x[:, step+1] - torque_obs[:,0] 
        x[:, step+1] = x[:, step+1] % gridSize

        y[:, step+1] = y[:,step] +  v * v_hat[:,1] 
        y[:, step+1] = y[:,step+1] + force_rob[:,1] 
        y[:, step+1] = y[:, step+1] + torque_rob[:,1] 
        y[:, step+1] = y[:,step+1] - force_item[:,1] 
        y[:, step+1] = y[:,step+1] + force_obs[:,1] 
        y[:, step+1] = y[:,step+1] + torque_obs[:,1] 
        #y[:, step+1] = y[:,step+1] - torque_obs[:,1] 
        y[:, step+1] = y[:,step+1] % gridSize

        
        # we only need to exclude robots
        pos = np.hstack((x[:, step+1].reshape((nOfRobots,1)), 
                         y[:, step+1].reshape((nOfRobots,1))))


        # here we'll pass by reference (but in the pytonic way)
        # esentially this means that volume exclusion will update the x and y array
        # which exists as the code is running, i.e. they won't be copied when entering to 
        # the function
        # think of this function as simply having the code put in another place, not
        # really a function with inputs and outputs
        volumeExclusion(x, y, pos, step, robRobNeig, robObsNeig, particle_radius, nOfRobots, obstacleRadius)
        # this will be used for interpretable animation
        robot_statesPerTime[:,step] = robot_states 
    return x, y, nOfCollectedItemsPerTime, item_positions_listPerTime



#nOfRobots = 30
nOfRobots = 1
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
nis = [np.pi *2, np.pi * 0.02, np.pi * 0.002]
ni = nis[1] 
ni = 0
v = 0.03
#v = 0.3
# Total time.

# TODO PLAY WITH THESE VALUES SEE WHAT HAPPENS TODO
obstacleRadius = 30
gridSize = 500
T0 = 1
particle_radius = 3
#torque_radius = 20 
torque_radius = 50
FI0 = 0.11#0.5
FR0 = 1
FO0 = 0.5
deviation = 0.55

nOfUnstuckingSteps = 600
stuckThresholdTime = 200
stuckThresholdDistance = v * 6

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 6000
# Initial values of x.
x = np.zeros((1 * nOfRobots,N+1))
#x[:,0] = 2 * np.random.random(nOfRobots) - 1 + gridSize // 2
x[:,0] = gridSize // 2 
y = np.zeros((1 * nOfRobots,N+1))
#y[:,0] = 2 * np.random.random(nOfRobots) - 1 + gridSize // 2
y[:,0] = gridSize // 2 - 3*obstacleRadius
robot_statesPerTime = np.zeros((1 * nOfRobots,N+1))


# 5 items
#nOfItems = 50
nOfItems = 0

#percetangeOfCoverage = 0.01
percetangeOfCoverage = 0.01
delivery_station = np.array([0,0])

#obstacles = initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius, delivery_station)
obstacles = np.array([[gridSize // 2 - 1.5 * obstacleRadius, gridSize // 2], 
                     [gridSize // 2 + 1.5 * obstacleRadius, gridSize // 2],
                     [gridSize // 2 - 3.5 * obstacleRadius, gridSize // 2],
                     [gridSize // 2, gridSize // 2],
                     [gridSize // 2 + 3.5 * obstacleRadius, gridSize // 2]])

item_positions_set, item_positions_list = initializeItems(nOfItems, gridSize, obstacles, obstacleRadius)

#walkType = 'levyFlight'
walkType = 'activeSwimming'
#walkType = 'brownianMotion'


x, y, nOfCollectedItemsPerTime, item_positions_listPerTime = \
    swimmersInFields(x, y, item_positions_set, delivery_station, N, nOfRobots, gridSize,  robot_statesPerTime, # sim params
                   v, particle_radius, torque_radius, obstacles,obstacleRadius,         # environment robot physical params 
                   walkType, ni, trans_dif_T, rot_dif_T, deviation,                                          # random walk params
                   T0, FR0, FI0, FO0,                                                    # artificial potential field parameters 
                   nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance)       # unstucking parameters

fig1, ax1 = plt.subplots(1)
ax1.grid()
# Plot the 2D trajectory.
# the camera way
camera = Camera(fig1)

# item_positions_list changes, you need to send a list of lists to know the changes
animate(x, y, robot_statesPerTime, item_positions_list, N, nOfRobots, particle_radius, ax1, camera, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius)


animation = camera.animate()
#animation.save('./vids/building_blocks/only_walks_and_APF_obs' + str(len(obstacles)) + '_item_' + str(nOfItems) + '_robots_' + str(nOfRobots) + '.mp4')
animation.save('testtest' + '.mp4')


# TODO generate artificial field plot as well
plt.show()

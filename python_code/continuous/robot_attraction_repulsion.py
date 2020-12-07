import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from movementFuns import *
from visualize import *
from environments import *


# to turn this into simple random motion, just don't add the torques (i.e. have only 
# the random portion of fi change)
def activeSwimmers(x, y, fi, item_positions_set, delivery_station, n, T0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, FR0, FI0, FW0, obstacles, 
        obstacleRadius, out=None):

    nOfCollectedItemsPerTime = [[0,0]]
    item_positions_list = np.array(list(item_positions_set))
    item_positions_listPerTime = [[0, item_positions_list]]
    nOfItems = len(item_positions_set)

    # 0 is search, 1 is delivering, 2 is going to pick up a spotted item
    robot_states = np.zeros(nOfRobots)
    robot_storage = {rob:[] for rob in range(nOfRobots)}
    for step in range(n):

        rand = (np.random.random((nOfRobots, 1)) - 0.5) * ni
        randFactors = np.array([rand[r] if robot_states[r] == 0 else 0.0 for r in range(nOfRobots)])

        pos = np.hstack((x[:, step].reshape((nOfRobots,1)), 
                         y[:, step].reshape((nOfRobots,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
        v_hat = np.hstack((np.cos(fi[:, step]) .reshape((nOfRobots,1)), 
                            np.sin(fi[:, step]).reshape((nOfRobots,1))))

        # get nbhds
        robRobNeig, robItemNeig, robObsNeig = getNeighbourhoods(pos, item_positions_list, 
                nOfRobots, nOfItems, particle_radius, torque_radius, obstacles, obstacleRadius)

        # the hw3 model is not good for this
        force_rob, force_item, force_obs = calcForceAttractionRepulsion(v, pos, robot_states, robRobNeig, robItemNeig, robObsNeig,  v_hat, nOfRobots, nOfItems, particle_radius, obstacleRadius)

        #print(force_rob)
        #print(force_item)
        force_rob = FR0 * force_rob
        force_item = FI0 * force_item 
        force_obs = FW0 * force_obs

        fi[:, step+1] = fi[:, step] + randFactors.reshape((nOfRobots,))

        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfRobots,1))) , 
                            np.sin(fi[:, step+1].reshape((nOfRobots,1)))))

        # TODO MAKE THIS CLEANER!!
        # check whether you're at the delivery station while you're at it
        # combine it all into a single for loop so that the states are consistent!!!
        v_hats2DelSt = v_hat2DeliveryStationFromState(pos, delivery_station, particle_radius, robot_states, nOfRobots)
        for robo in v_hats2DelSt:
            isDone = (v_hats2DelSt[robo] == 0).all()
            if isDone:
                nOfCollectedItemsPerTime.append([step, nOfCollectedItemsPerTime[-1][1] + len(robot_storage[robo])])
                robot_storage[robo].clear()
                robot_states[robo] = 0
            else:
                v_hat[robo] = v_hats2DelSt[robo]

        # TODO make this thing work!!!!!!!!!!!!!1
        # check if they got items in their visibility sphere
        robsWithNearItems = v_hat2NearItem(pos, robItemNeig, particle_radius, nOfRobots, robot_states)
#        # returns either vector (ndarray) to item or tuple denoting item position if it has been picked up
        for robo in robsWithNearItems:
            #print(robsWithNearItems[robo])
            if type(robsWithNearItems[robo]) == tuple:
                robot_storage[robo].append(robsWithNearItems[robo])
                item_positions_set.remove(robsWithNearItems[robo])
                nOfItems -= 1
                robot_states[robo] = 1

                item_positions_list = np.array(list(item_positions_set))
                item_positions_listPerTime.append([step, item_positions_list])
            else:
                v_hat[robo] = robsWithNearItems[robo]
                robot_states[robo] = 2



        # if the robot is in the delivery state, v_hat is the direction to the delivery station

        x[:, step+1] = (x[:,step] +  v * v_hat[:,0]+ force_rob[:,0] - force_item[:,0] + force_obs[:,0]) % gridSize
        #x[:, step+1] = (x[:,step] +  v * v_hat[:,0]+ force_rob[:,0] - force_item[:,0] ) % gridSize
        y[:, step+1] = (y[:,step] +  v * v_hat[:,1] + force_rob[:,1] - force_item[:,1] + force_obs[:,1]) % gridSize
        #y[:, step+1] = (y[:,step] +  v * v_hat[:,1] + force_rob[:,1] - force_item[:,1] ) % gridSize
        #x[:, step+1] = (x[:,step] +  v * v_hat[:,0] - force_item[:,0]) % gridSize
        #y[:, step+1] = (y[:,step] +  v * v_hat[:,1] - force_item[:,1]) % gridSize

        
        # we only need to exclude robots, items will most likely be picked up anyway
        pos = np.hstack((x[:, step+1].reshape((nOfRobots,1)), 
                         y[:, step+1].reshape((nOfRobots,1))))


        # here we'll pass by reference (but in the pytonic way)
        # esentially this means that volume exclusion will update the x and y array
        # which exists as the code is running, i.e. they won't be copied when entering to 
        # the function
        # think of this function as simply having the code put in another place, not
        # really a function with inputs and outputs
        volumeExclusion(x, y, pos, step, robRobNeig, robObsNeig, particle_radius, nOfRobots, obstacleRadius)
            
#        item_positions_list, nOfItems = handleItems(x, y, step, robItemNeig, pos, robot_storage, 
#            robot_states, item_positions_set, item_positions_listPerTime, particle_radius, nOfRobots, nOfItems)

        

    return x, y, nOfCollectedItemsPerTime, item_positions_listPerTime




nOfRobots = 20
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
nis= [np.pi *2, np.pi * 0.2, np.pi * 0.002]
ni = nis[-1] 
#v = 0.05
v = 0.1
# Total time.
obstacleRadius = 30
gridSize = 1000
torque0 = 1
particle_radius = 5
torque_radius = 100 
FI0 = 1
FR0 = 10
FW0 = 1

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 14000
# Initial values of x.
# you should init this to sth other than 0
x = np.zeros((1 * nOfRobots,N+1))
#x[:,0] = np.random.random(nOfRobots) * gridSize
x[:,0] = 2 * np.random.random(nOfRobots) - 1 + 500
y = np.zeros((1 * nOfRobots,N+1))
#y[:,0] = np.random.random(nOfRobots) * gridSize
y[:,0] = 2 * np.random.random(nOfRobots) - 1 + 500
fi = np.zeros((1 * nOfRobots,N+1))
fi[:,0] = np.random.random(nOfRobots) * 2*np.pi
#x[:, 0] = 0.0


# 5 items
nOfItems = 50

percetangeOfCoverage = 0.01
obstacles = initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius)

item_positions_set, item_positions_list = initializeItems(nOfItems, gridSize, obstacles)

delivery_station = np.array([500,500])


x, y, nOfCollectedItemsPerTime, item_positions_listPerTime = activeSwimmers(x, y, fi, item_positions_set, delivery_station, N, torque0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, 
    particle_radius, torque_radius, FR0, FI0, FW0, obstacles, obstacleRadius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (3*(ax.get_window_extent().width  / (gridSize+1.) * 72./fig.dpi) ** 2)

# item_positions_list changes, you need to send a list of lists to know the changes
visualise(x, y, item_positions_list, N, nOfRobots, particle_radius, ax, camera, s, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius)


animation = camera.animate()
animation.save('robot_roam=' +str(ni)  +'.mp4')
plt.show()

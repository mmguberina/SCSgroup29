import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from movementFuns import *
from animate import *


# to turn this into simple random motion, just don't add the torques (i.e. have only 
# the random portion of fi change)
def activeSwimmers(x, y, fi, item_positions_set, delivery_station, n, T0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

    # keep track of item changes
    nOfCollectedItemsPerTime = [[0,0]]
    item_positions_list = np.array(list(item_positions_set))
    item_positions_listPerTime = [[0, item_positions_list]]
    nOfItems = len(item_positions_set)

    # 0 is search, 1 is delivering, 2 is ready to drop off item
    robot_states = np.zeros(nOfRobots)
    robot_storage = {rob:[] for rob in range(nOfRobots)}
    for step in range(n):
        
        # generate random factors
        rand = (np.random.random((nOfRobots, 1)) - 0.5) * ni
        randFactors = np.array([rand[r] if robot_states[r] == 0 else 0.0 for r in range(nOfRobots)])
        # we need x and y to form a vector to perform the computations
        pos = np.hstack((x[:, step].reshape((nOfRobots,1)), 
                         y[:, step].reshape((nOfRobots,1))))

        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
        v_hat = np.hstack((np.cos(fi[:, step]) .reshape((nOfRobots,1)), 
                            np.sin(fi[:, step]).reshape((nOfRobots,1))))

        # get nbhds
        robRobNeig, robItemNeig = getNeighbourhoods(pos, item_positions_list, 
                nOfRobots, nOfItems, particle_radius, torque_radius)


        torque_rob, torque_item = calcTorqueFromNeigh(pos, robot_states, robRobNeig, robItemNeig,                               v_hat, nOfRobots, nOfItems)

        torque_rob = T0 * torque_rob
        torque_item = T0 * torque_item

        # update the orientation
        fi[:, step+1] = fi[:, step] + torque_item.reshape((nOfRobots,)) \
                            - torque_rob.reshape((nOfRobots,)) \
                            + randFactors.reshape((nOfRobots,))

        # calculate the velocities according to random walking
        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfRobots,1))), 
                            np.sin(fi[:, step+1].reshape((nOfRobots,1)))))

        # we however also calculate velocities for going to the delivery station
        v_hat2DelSt = v_hat2DeliveryStation(pos, \
                                    delivery_station, particle_radius).reshape((nOfRobots,2))
        
        # assign the correct velocities according to the robot's states
        v_hat = np.array([v_hat[v] if robot_states[v]==0 else v_hat2DelSt[v] \
                          for v in range(nOfRobots)]).reshape((nOfRobots,2))

        # check whether robots are the delivery station
        isDone = (v_hat == 0).all(axis=1)
        robot_states = np.array([ 0 if isDone[i] else robot_states[i] \
                                for i in range(nOfRobots)])
        # if yes empty their storage and change their state back to 0 (search)
        for robo in range(nOfRobots):
            if isDone[robo]:
                nOfCollectedItemsPerTime.append([step, nOfCollectedItemsPerTime[-1][1] + len(robot_storage[robo])])
                robot_storage[robo].clear()


        # if the robot is in the delivery state, v_hat is the direction to the delivery station

        x[:, step+1] = (x[:,step] +  v * v_hat[:,0]) % gridSize
        y[:, step+1] = (y[:,step] +  v * v_hat[:,1]) % gridSize

        
        # we only need to exclude robots, items will most likely be picked up anyway
        pos = np.hstack((x[:, step+1].reshape((nOfRobots,1)), 
                         y[:, step+1].reshape((nOfRobots,1))))


        # here we'll pass by reference (but in the pytonic way)
        # esentially this means that volume exclusion will update the x and y array
        # which exists as the code is running, i.e. they won't be copied when entering to 
        # the function
        # think of this function as simply having the code put in another place, not
        # really a function with inputs and outputs
        volumeExclusion(x, y, pos, step, robRobNeig, particle_radius, nOfRobots)
            
        item_positions_list, nOfItems = handleItems(x, y, step, robItemNeig, pos, robot_storage, 
            robot_states, item_positions_set, item_positions_listPerTime, particle_radius, nOfRobots, nOfItems)

    return x, y, nOfCollectedItemsPerTime, item_positions_listPerTime




nOfRobots = 10
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
nis= [np.pi *2, np.pi * 0.02, np.pi * 0.002]
ni = nis[1] 
#v = 0.05
v = 0.5
# Total time.
gridSize = 1000
torque0 = 1
particle_radius = 5
torque_radius = 50 * particle_radius

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 10000
# Time step size
# Initial values of x.
# you should init this to sth other than 0
x = np.zeros((1 * nOfRobots,N+1))
x[:,0] = np.random.random(nOfRobots) * gridSize
y = np.zeros((1 * nOfRobots,N+1))
y[:,0] = np.random.random(nOfRobots) * gridSize
fi = np.zeros((1 * nOfRobots,N+1))
fi[:,0] = np.random.random(nOfRobots) * 2*np.pi
#x[:, 0] = 0.0


# 5 items
nOfItems = 5
item_positions_list = np.fix(np.random.random((nOfItems, 2)) * gridSize)
item_positions_set = set(map(tuple, item_positions_list))

delivery_station = np.array([500,500])

x, y, nOfCollectedItemsPerTime, item_positions_listPerTime = activeSwimmers(x, y, fi, item_positions_set, delivery_station, N, torque0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)


fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (3*(ax.get_window_extent().width  / (gridSize+1.) * 72./fig.dpi) ** 2)

# item_positions_list changes, you need to send a list of lists to know the changes
visualise(x, y, item_positions_list, N, nOfRobots, particle_radius, ax, camera, s, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station)


animation = camera.animate()
animation.save('robot_roam=' +str(ni)  +'.mp4')
plt.show()

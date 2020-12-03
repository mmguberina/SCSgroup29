import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from movementFuns import *
from visualize import *


# to turn this into simple random motion, just don't add the torques (i.e. have only 
# the random portion of fi change)
def activeSwimmers(x, y, fi, item_positions_set, delivery_station, n, dt, T0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

    nOfCollectedItemsPerTime = [0]

    item_positions_list = np.array(list(item_positions_set))
    nOfItems = len(item_positions_set)
    # 0 is search, 1 is delivering, 2 is ready to drop off item
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
        # init torque
        torque = np.zeros((nOfRobots,1))
        torque_item = np.zeros((nOfRobots,1))


        # the hw3 model is not good for this
        torque, torque_item, robRobNeig, robItemNeig = calcTorque(pos, robot_states, 
                item_positions_list, v_hat, nOfRobots, nOfItems,
                particle_radius, torque_radius)

        torque = T0 * torque
        torque_item = T0 * torque_item
        fi[:, step+1] = fi[:, step] + torque_item.reshape((nOfRobots,)) \
                            - torque.reshape((nOfRobots,)) \
                            + randFactors.reshape((nOfRobots,))

        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfRobots,1))), 
                            np.sin(fi[:, step+1].reshape((nOfRobots,1)))))

        

        # check whether you're at the delivery station while you're at it
        v_hat2DelSt = v_hat2DeliveryStation(pos, \
                                    delivery_station, particle_radius).reshape((nOfRobots,2))
        # check whether robots are the delivery station
        isDone = (v_hat2DelSt == 0).all(axis=1)
        robot_states = np.array([ 0 if isDone[i] else robot_states[i] \
                                for i in range(nOfRobots)])
        # if yes empty their storage and change their state back to 0 (search)
        nOfDelivered = nOfCollectedItemsPerTime[-1]
        for robo in range(nOfRobots):
            if isDone[robo]:
                nOfDelivered += len(robot_storage[robo])
                robot_storage[robo].clear()
        nOfCollectedItemsPerTime.append(nOfDelivered)

        v_hat = np.array([v_hat[v] if robot_states[v]==0 else v_hat2DelSt[v] \
                          for v in range(nOfRobots)]).reshape((nOfRobots,2))

        # if the robot is in the delivery state, v_hat is the direction to the delivery station

        x[:, step+1] = (x[:,step] +  v * v_hat[:,0]) % gridSize
        y[:, step+1] = (y[:,step] +  v * v_hat[:,1]) % gridSize

        
        # we only need to exclude robots, items will most likely be picked up anyway
        pos = np.hstack((x[:, step+1].reshape((nOfRobots,1)), 
                         y[:, step+1].reshape((nOfRobots,1))))


        

        # volume exclusion
        # and item picking while we're at it
        for p in range(nOfRobots):
            r = pos[p] - pos 
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
            #r_item = pos[p] - item_positions_list
            #rnorms_item = np.linalg.norm(r_item, axis=1)
            rnorms[p] = 'Inf'
            r_hat  = r / rnorms

            for n in robRobNeig[p]:
                if rnorms[n] < 2 * particle_radius:
                    overlap = 2 * particle_radius - rnorms[n]
                    moveVec = r_hat[n] * (overlap / 2)
#                    print(moveVec)
                    x[p, step+1] += moveVec[0]
                    y[p, step+1] += moveVec[1]
                    x[n, step+1] -= moveVec[0]
                    y[n, step+1] -= moveVec[1]
            
# you'll need to somehow keep track of item changes so that they can be plotted in the simulation
            for n in robItemNeig[p]:
                rnorm_item = np.linalg.norm(np.array(n) - pos[p])
                if rnorm_item < 2 * particle_radius:
                    item_positions_set.remove(n)
                    nOfItems -= 1
                    if nOfItems == 0:
                        return x, y, nOfCollectedItemsPerTime
                    item_positions_list = np.array(list(item_positions_set))
                    robot_storage[p].append(n)
                    robot_states[p] = 1

    return x, y, nOfCollectedItemsPerTime




nOfRobots = 30
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
nis= [np.pi *2, np.pi * 0.2, np.pi * 0.002]
ni = nis[2] 
v = 0.05
# Total time.
T = 50
gridSize = 100
torque0 = 1
particle_radius = 1
torque_radius = 3 * particle_radius

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 2000
# Time step size
dt = T/N
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
item_positions_list = np.fix(np.random.random((5, 2)) * gridSize)
item_positions_set = set(map(tuple, item_positions_list))

delivery_station = np.array([30,30])

x, y, nOfCollectedItemsPerTime = activeSwimmers(x, y, fi, item_positions_set, delivery_station, N, dt, torque0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (3*(ax.get_window_extent().width  / (gridSize+1.) * 72./fig.dpi) ** 2)

# item_positions_list changes, you need to send a list of lists to know the changes
visualise(x, y, item_positions_list, N, nOfRobots, particle_radius, ax, camera, s, nOfCollectedItemsPerTime)


animation = camera.animate()
animation.save('robot_roam=' +str(ni)  +'.mp4')
plt.show()

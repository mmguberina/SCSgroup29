import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from calcTorque import *
from visualize import *


# to turn this into simple random motion, just don't add the torques (i.e. have only 
# the random portion of fi change)
def activeSwimmers(x, y, fi, item_positions_set, delivery_station, n, dt, T0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

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

        # TODO check whether you're at the delivery station while you're at it
        v_hat2DeliveryStation = v_hat2DeliveryStation(pos, \
                                    delivery_station).reshape((nOfRobots,2))

        v_hat = np.array([v_hat[v] if robot_states[v]==0 else v_hat2DeliveryStation[v] \
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
            r_item = pos[p] - item_positions_list
            rnorms_item = np.linalg.norm(r_item, axis=1)
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
                if rnorms_item[n] < 2 * particle_radius:
                    item_positions_set.remove(n)
                    item_positions_list = np.array(list(item_positions_set))
                    robot_storage[p].append(n)
                    robot_states[p] = 1

    return x, y




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

x, y = activeSwimmers(x, y, fi, item_positions_set, delivery_station, N, dt, torque0, nOfRobots, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (3*(ax.get_window_extent().width  / (gridSize+1.) * 72./fig.dpi) ** 2)

visualise(x, y, item_positions, N, nOfRobots, particle_radius, ax, camera, s)


#plt.show()
# Mark the start and end points.
#ax.plot(x[0,0],x[1,0], 'go')
#ax.plot(x[0,-1], x[1,-1], 'ro')
#
## More plot decorations.
#ax.set_title('2D Brownian Motion')
#ax.set_xlabel('x', fontsize=16)
#ax.set_ylabel('y', fontsize=16)
#ax.axis('equal')
#t = np.linspace(0,T,int(T//dt))
#msd = MSD(x, dt, T)
#msd = calcMSDAvg(rot_dif_T, trans_dif_T, v, T, N)
#ax.loglog(t, msd)
animation = camera.animate()
animation.save('robot_roam=' +str(ni)  +'.mp4')
plt.show()

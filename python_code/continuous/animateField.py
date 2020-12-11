import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from animate import *
from environments import *
from runSim import *



### outline:
# create x, y = meshgrid,
# z = zeros.
#
# loop over robots,
# calculate fields within their radius (and outside their volume)
# add field to x,y that are within their radius
#
# loop over items,
# calculate fields within their radius (and outside their volume)
# add field to x,y that are within their radius
# loop over environments,
# calculate fields within their radius (and outside their volume)
# add field to x,y that are within their radius


import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection

def animateField(x, y, robot_statesPerTime, item_positions, N, nOfRobots, particle_radius, ax, camera, gridSize, fieldResolution, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius):
    item_positions_listPerTime.reverse()
    nOfCollectedItemsPerTime.reverse()
    currently_collected = 0
    # in case there won't be any, otherwise it's assigned below
    item_positions = []

    for timestep in range(N):
       # for i in range(nOfRobots):
        if timestep % 40 == 0:

            if len(item_positions_listPerTime) > 0 and timestep >= item_positions_listPerTime[-1][0]:
                item_positions = item_positions_listPerTime.pop()[1]

            if len(nOfCollectedItemsPerTime) > 0 and timestep >= nOfCollectedItemsPerTime[-1][0]:
                currently_collected = nOfCollectedItemsPerTime.pop()[1]

            breakpoint()
            clusteredP = set()
            cluster2 = set()
            actActNeig = {i:[] for i in range(nOfRobots)}

            inx, iny = np.meshgrid(np.linspace(0, gridSize, fieldResolution),
                                   np.linspace(0, gridSize, fieldResolution))
            u = np.zeros(inx.shape)
            v = np.zeros(iny.shape)

            pos = np.hstack((x[:, timestep].reshape((nOfRobots,1)),
                             y[:, timestep].reshape((nOfRobots,1))))

            robRobNeig, robItemNeig, robObsNeig = getNeighbourhoods(pos, item_positions_listPerTime,
                nOfRobots, nOfItems, particle_radius, torque_radius, obstacles, obstacleRadius)
            for i in range(nOfRobots):
                r = pos[i] - pos
                rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
                cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfRobots) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})

                u, v = calcRobField(pos[i], particle_radius, torque_radius, robRobNeig, robItemNeig, robObsNeig,
                                    inx, iny, u, v)
            for obstacle in obstacles:
                circle = Circle(tuple(obstacle), obstacleRadius, color='black')
                ax.add_patch(circle)
                u, v = calcObsField(obstacle, pos, inx, iny, u, v)

            i = 0
            robot_states = robot_statesPerTime[:,timestep]
            for robot in zip(x[:,timestep], y[:,timestep]):
                circle = Circle(robot, particle_radius, color='red')
                if robot_states[i] == 0:
                    color_vs = 'wheat'
                if robot_states[i] == 1:
                    color_vs = 'green'
                if robot_states[i] == 2:
                    color_vs = 'lightsalmon'
                if robot_states[i] == 3:
                    color_vs = 'darkred'
                if robot_states[i] == 4:
                    color_vs = 'goldenrod'
                visSphere = Circle(robot, torque_radius, alpha=0.3, color=color_vs)
                ax.add_patch(visSphere)
                ax.add_patch(circle)
                i += 1

            if len(item_positions) > 0:
                for item in zip(item_positions[:,0], item_positions[:,1]):
                    circle = Circle(item, particle_radius, color='green')
                    ax.add_patch(circle)

            if len(cluster2) > 0:
                for clst in cluster2:
                    clst = Circle(clst, particle_radius, color='blue')
                    ax.add_patch(circle)

            circle = Circle(tuple(delivery_station), particle_radius, facecolor='yellow', edgecolor='black')
            ax.add_patch(circle)

            ax.axis('scaled')

            #ax.scatter(walls[:,0], walls[:,1], s=s, color='black')
                #ax.scatter(cluster2[:, 0], cluster2[:, 1], color='blue')
            ax.set_title("total delivered items: " + str(currently_collected))
            camera.snap()


def calcRobField(robot, particleRadius, torqueRadius, robRobNeig, robItemNeig, robObsNeig, inx, iny, u, v):
    xmin = robot[0] - torqueRadius
    xmax = robot[0] + torqueRadius
    ymin = robot[1] - torqueRadius
    ymax = robot[1] + torqueRadius
    # xToUse, yToUse = {all points within torqueRadius of robot}
    # calculate field in points (xToUse, yToUse)
    # make u', v'
    # u += u'
    # v += v'

    rx = (inx - robot[0])**2
    ry = (iny - robot[1])**2
    r = np.sqrt(rx + ry)
    iToUse, jToUse = np.where(r < torqueRadius)
    xToUse = inx[iToUse, jToUse]
    yToUse = iny[iToUse, jToUse]

    # TODO put the coefficients in the force fuctions
    # and make them < v! (otherwise they will "discontinuously" jump)
    # TODO check signs, strengths and other stuff
    force_rob = FR0 * calcForceRob(v, pos, robRobNeig, nOfRobots, particle_radius)
    torque_rob = FR0 * calcTorqueRob_as_v(v, pos, robRobNeig, v_hat, nOfRobots, particle_radius)

    force_item = FI0 * calcForceItem(v, pos, robItemNeig, nOfRobots, particle_radius)

    force_obs = FO0 * calcForceObs(v, pos, robObsNeig,  nOfRobots, particle_radius, obstacleRadius)

    torque_obs = FO0 * calcTorqueObs_as_v(v, pos, robObsNeig, v_hat, nOfRobots, obstacleRadius)
    currfield = force_rob + torque_rob - force_item - force_obs - torque_obs
    breakpoint()
    #u += 

###    torque_rob = calcTorqueRob(robot, robot_states, robRobNeig, v_hat, nOfRobots)
###    force_rob = calcForceRob(v, robot, robot_states, robRobNeig, v_hat, nOfRobots, particlRadius)
###    # calculate direction vector to things in neighbourhood
###    r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
###    # calculate the norm of every direction vector
###    rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
###    # collect only nearby ones
###    # we need rhat
###    r_item_hat  = r_item / rnorms_item
###    # dot 'em. dot does not support axis thing so we do it like this
###    dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItemsInNeigh,1))
###    coefs_item = dots_item / rnorms_item**2
###    # try repelling them now
###    # crosses v_i with r_i and does so for all i
###    crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItemsInNeigh, 1))
###    particle_torques_item = coefs_item * crosses_item
###
###    torque_item[i] = np.sum(particle_torques_item)

    ### # assume fieldx, fieldy are the x and y components of the field
    ### u += fieldx
    ### v += fieldy
    return u, v

def calcObsField(obstacle, pos, inx, iny, u, v):
    return u, v



def drawPotentialFields(v, pos, robot_states, robRobNeig, v_hat, nOfRobots, particleRadius, obstacleRadius, torqueRadius, FR0, FO0, obstacles, meshgrid):
    x, y = meshgrid
    u = np.zeros(x.shape)
    v = np.zeros(y.shape)

    for i in range(nOfRobots):
        # calculate fields
        breakpoint()
        robPos = pos[:,i]
        torque_rob = calcTorqueRob(robPos, robot_states, robRobNeig, v_hat, nOfRobots)
        force_rob = calcForceRob(v, robPos, robot_states, robRobNeig, v_hat, nOfRobots, particlRadius)
        # calculate direction vector to things in neighbourhood
        r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
        # calculate the norm of every direction vector
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
        # collect only nearby ones
        # we need rhat
        r_item_hat  = r_item / rnorms_item
        # dot 'em. dot does not support axis thing so we do it like this
        dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItemsInNeigh,1))
        coefs_item = dots_item / rnorms_item**2
        # try repelling them now
        # crosses v_i with r_i and does so for all i
        crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItemsInNeigh, 1))
        particle_torques_item = coefs_item * crosses_item

        torque_item[i] = np.sum(particle_torques_item)

        ### # assume fieldx, fieldy are the x and y components of the field
        ### u += fieldx
        ### v += fieldy


    #force_rob = FR0 * calcForceRob(v, pos, robot_states, robRobNeig, v_hat, nOfRobots, particleRadius)
    #force_obs = FO0 * calcForceObs(v, pos, robot_states, robObsNeig, v_hat, nOfRobots, obstacleRadius)


    return u, v




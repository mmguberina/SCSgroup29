import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib
from animate import *
from environments import *
from runSim import *



import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection

def animateField(x, y, v, robot_statesPerTime, item_positions, N, nOfRobots, particle_radius, ax, camera, gridSize, fieldResolution, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius, forceandtorquecoeffs):
    item_positions_listPerTime.reverse()
    nOfCollectedItemsPerTime.reverse()
    currently_collected = 0
    # in case there won't be any, otherwise it's assigned below
    item_positions = []

    for timestep in range(1, N):
        if timestep % 40 == 0:
            if len(item_positions_listPerTime) > 0 and timestep >= item_positions_listPerTime[-1][0]:
                item_positions = item_positions_listPerTime.pop()[1]

            nOfItems = len(item_positions)
            if len(nOfCollectedItemsPerTime) > 0 and timestep >= nOfCollectedItemsPerTime[-1][0]:
                currently_collected = nOfCollectedItemsPerTime.pop()[1]

            clusteredP = set()
            cluster2 = set()
            actActNeig = {i:[] for i in range(nOfRobots)}

            gridx, gridy = np.meshgrid(np.linspace(0, gridSize, fieldResolution),
                                       np.linspace(0, gridSize, fieldResolution))

            fieldx = np.zeros(gridx.shape)
            fieldy = np.zeros(gridy.shape)

            pos = np.hstack((x[:, timestep].reshape((nOfRobots,1)),
                             y[:, timestep].reshape((nOfRobots,1))))


            v = np.array([x[:, timestep] - x[:, timestep-1],
                          y[:, timestep] - y[:, timestep-1]])

            for i in range(nOfRobots):
                r = pos[i] - pos
                rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
                cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfRobots) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})

                fieldx, fieldy = calcRobField(pos[i], v[:,i], particle_radius, torque_radius, forceandtorquecoeffs, gridx, gridy, fieldx, fieldy)
            for obstacle in obstacles:
                circle = Circle(tuple(obstacle), obstacleRadius, color='black')
                ax.add_patch(circle)
                fieldx, fieldy = calcObsField(obstacle, obstacleRadius, torque_radius, forceandtorquecoeffs, gridx, gridy, fieldx, fieldy)

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
                    fieldx, fieldy = calcItemField(item, torque_radius, forceandtorquecoeffs, gridx, gridy, fieldx, fieldy)

            if len(cluster2) > 0:
                for clst in cluster2:
                    clst = Circle(clst, particle_radius, color='blue')
                    ax.add_patch(circle)

            circle = Circle(tuple(delivery_station), particle_radius, facecolor='yellow', edgecolor='black')
            ax.add_patch(circle)

            ax.axis('scaled')
            ax.quiver(gridx, gridy, fieldx, fieldy, color='black', alpha=1)

            #ax.scatter(walls[:,0], walls[:,1], s=s, color='black')
                #ax.scatter(cluster2[:, 0], cluster2[:, 1], color='blue')
            ax.set_title("total delivered items: " + str(currently_collected))
            camera.snap()


def calcItemField(item, torqueRadius, forceandtorquecoeffs, gridx, gridy, fieldx, fieldy):
    FR0, FI0, FO0, T0 = forceandtorquecoeffs

    # find area to work in
    # TODO: I feel like this can be done more efficiently
    # currently we calculate r twice, here and then again a little later
    rx = (gridx - item[0])**2
    ry = (gridy - item[1])**2
    r = np.sqrt(rx + ry)
    iToUse, jToUse = np.where(r < torqueRadius)
    xToUse = gridx[iToUse, jToUse]
    yToUse = gridy[iToUse, jToUse]

    # some vectors etc we need
    pointsInField = np.vstack((xToUse, yToUse)).transpose() # make sure it's useable
    nOfPoints = len(xToUse)
    rToUse = pointsInField - item # vectors, outward from item
    rnorms = np.linalg.norm(rToUse, axis=1).reshape((len(xToUse),1))
    rhat  = rToUse / rnorms


    # #calculating torques
    # vnorm = np.linalg.norm(v)
    # vhat = v / vnorm
    # dots = np.sum(vhat * rhat)
    # coefs = dots / (rnorms - particleRadius) # i'll be honest, this is the last step I feel I understand
    # crosses = np.cross(vhat, rhat)
    # particle_torques = coefs * crosses
    # torquesum = np.sum(particle_torques, axis=1).reshape((nOfPoints, 1))
    # torques = vnorm * np.array([np.cos(torquesum), np.sin(torquesum)]).reshape((nOfPoints, 2))

    # add forces and torques
    fieldx[iToUse, jToUse] += FI0 * rhat[:,0]
    fieldy[iToUse, jToUse] += FI0 * rhat[:,1]
    # fieldx[iToUse, jToUse] += FI0 * torques[:,0]
    # fieldy[iToUse, jToUse] += FI0 * torques[:,1]

    return fieldx, fieldy

def calcRobField(robot, v, particleRadius, torqueRadius, forceandtorquecoeffs, gridx, gridy, fieldx, fieldy):
    FR0, FI0, FO0, T0 = forceandtorquecoeffs

    # find area to work in
    # TODO: I feel like this can be done more efficiently
    # currently we calculate r twice, here and then again a little later
    rx = (gridx - robot[0])**2
    ry = (gridy - robot[1])**2
    r = np.sqrt(rx + ry)
    iToUse, jToUse = np.where(np.logical_and(r < torqueRadius, r > particleRadius))
    xToUse = gridx[iToUse, jToUse]
    yToUse = gridy[iToUse, jToUse]

    # some vectors etc we need
    pointsInField = np.vstack((xToUse, yToUse)).transpose() # make sure it's useable
    nOfPoints = len(xToUse)
    rToUse = pointsInField - robot # vectors, outward from robot
    rnorms = np.linalg.norm(rToUse, axis=1).reshape((len(xToUse),1))
    rhat  = rToUse / rnorms


    #calculating torques
    vnorm = np.linalg.norm(v)
    vhat = v / vnorm
    dots = np.sum(vhat * rhat)
    coefs = dots / (rnorms - particleRadius) # i'll be honest, this is the last step I feel I understand
    crosses = np.cross(vhat, rhat)
    particle_torques = coefs * crosses
    torquesum = np.sum(particle_torques, axis=1).reshape((nOfPoints, 1))
    torques = vnorm * np.array([np.cos(torquesum), np.sin(torquesum)]).reshape((nOfPoints, 2))

    # add forces and torques
    fieldx[iToUse, jToUse] += FR0 * rhat[:,0]
    fieldy[iToUse, jToUse] += FR0 * rhat[:,1]
    fieldx[iToUse, jToUse] += FR0 * torques[:,0]
    fieldy[iToUse, jToUse] += FR0 * torques[:,1]


    return fieldx, fieldy

def calcObsField(obstacle, obstacleRadius, torqueRadius, forceandtorquecoeffs, gridx, gridy, fieldx, fieldy):
    FR0, FI0, FO0, T0 = forceandtorquecoeffs

    rx = (gridx - obstacle[0])**2
    ry = (gridy - obstacle[1])**2
    r = np.sqrt(rx + ry)
    iToUse, jToUse = np.where(np.logical_and(r < torqueRadius, r > obstacleRadius))
    xToUse = gridx[iToUse, jToUse]
    yToUse = gridy[iToUse, jToUse]

    pointsInField = np.vstack((xToUse, yToUse)).transpose()
    rToUse = pointsInField - obstacle
    rnorms = np.linalg.norm(rToUse, axis=1).reshape((len(xToUse),1))
    rhat  = rToUse / rnorms
    fieldx[iToUse, jToUse] += FO0 * rhat[:,0]
    fieldy[iToUse, jToUse] += FO0 * rhat[:,1]

    # #calculating torques
    # vnorm = np.linalg.norm(v)
    # vhat = v / vnorm
    # dots = np.sum(vhat * rhat)
    # coefs = dots / (rnorms - particleRadius) # i'll be honest, this is the last step I feel I understand
    # crosses = np.cross(vhat, rhat)
    # particle_torques = coefs * crosses
    # torquesum = np.sum(particle_torques, axis=1).reshape((nOfPoints, 1))
    # torques = vnorm * np.array([np.cos(torquesum), np.sin(torquesum)]).reshape((nOfPoints, 2))

    # # add forces and torques
    # fieldx[iToUse, jToUse] += FR0 * rhat[:,0]
    # fieldy[iToUse, jToUse] += FR0 * rhat[:,1]
    # fieldx[iToUse, jToUse] -= FO0 * torques[:,0]
    # fieldy[iToUse, jToUse] -= FO0 * torques[:,1]

    return fieldx, fieldy





import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection


# THIS IS RIDICULOUSLY WRONG !!!!!!!!!!!!!!!!!!!!!!!
# thing is, you have to initialize all artistis (circles in our case) in a init function 
# and then you update those specific circles in the update function
# right now you're just adding new circles on top of old circles!!!!!!!!!!!

# but i'm not gonna bother with that rn, that's certainly not priority

def initAnim(x, y, robot_statesPerTime, N, nOfRobots, particle_radius, ax, item_positions, nOfCollectedItemsPerTime, 
        item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius, nOfSkippedFrames):

    allArtists = []
    timestep = frame * nOfSkippedFrames
    print("got called with", timestep)
    item_positions_listPerTime.reverse()
    nOfCollectedItemsPerTime.reverse()
    currently_collected = 0
    # in case there won't be any, otherwise it's assigned below


    if len(item_positions_listPerTime) > 0 and timestep >= item_positions_listPerTime[-1][0]:
        item_positions = item_positions_listPerTime.pop()[1]

    if len(nOfCollectedItemsPerTime) > 0 and timestep >= nOfCollectedItemsPerTime[-1][0]:
        currently_collected = nOfCollectedItemsPerTime.pop()[1]

    clusteredP = set()
    cluster2 = set()
    actActNeig = {i:[] for i in range(nOfRobots)}

    pos = np.hstack((x[:, timestep].reshape((nOfRobots,1)), 
                     y[:, timestep].reshape((nOfRobots,1))))

    for i in range(nOfRobots):
        r = pos[i] - pos 
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
        cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfRobots) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})

    for obstacle in obstacles:
        circle = Circle(tuple(obstacle), obstacleRadius, color='black')
        allArtists.append(ax.add_patch(circle))

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
        allArtists.append(ax.add_patch(visSphere))
        allArtists.append(ax.add_patch(circle))
        i += 1

    if len(item_positions) > 0:
        for item in zip(item_positions[:,0], item_positions[:,1]):
            circle = Circle(item, particle_radius, color='green')
            allArtists.append(ax.add_patch(circle))
    
    if len(cluster2) > 0:
        for clst in cluster2:
            clst = Circle(clst, particle_radius, color='blue')
            allArtists.append(ax.add_patch(circle))

    circle = Circle(tuple(delivery_station), particle_radius, facecolor='yellow', edgecolor='black')
    allArtists.append(ax.add_patch(circle))

    ax.axis('scaled')

    #ax.scatter(walls[:,0], walls[:,1], s=s, color='black')
        #ax.scatter(cluster2[:, 0], cluster2[:, 1], color='blue')
    ax.set_title("total delivered items: " + str(currently_collected))

    return allArtists


def frameUpdate(frame, x, y, robot_statesPerTime, N, nOfRobots, particle_radius, ax, item_positions, nOfCollectedItemsPerTime, 
        item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius, nOfSkippedFrames):
    allArtists = []
    timestep = frame * nOfSkippedFrames
    print("got called with", timestep)
    item_positions_listPerTime.reverse()
    nOfCollectedItemsPerTime.reverse()
    currently_collected = 0
    # in case there won't be any, otherwise it's assigned below


    if len(item_positions_listPerTime) > 0 and timestep >= item_positions_listPerTime[-1][0]:
        item_positions = item_positions_listPerTime.pop()[1]

    if len(nOfCollectedItemsPerTime) > 0 and timestep >= nOfCollectedItemsPerTime[-1][0]:
        currently_collected = nOfCollectedItemsPerTime.pop()[1]

    clusteredP = set()
    cluster2 = set()
    actActNeig = {i:[] for i in range(nOfRobots)}

    pos = np.hstack((x[:, timestep].reshape((nOfRobots,1)), 
                     y[:, timestep].reshape((nOfRobots,1))))

    for i in range(nOfRobots):
        r = pos[i] - pos 
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
        cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfRobots) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})

    for obstacle in obstacles:
        circle = Circle(tuple(obstacle), obstacleRadius, color='black')
        allArtists.append(ax.add_patch(circle))

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
        allArtists.append(ax.add_patch(visSphere))
        allArtists.append(ax.add_patch(circle))
        i += 1

    if len(item_positions) > 0:
        for item in zip(item_positions[:,0], item_positions[:,1]):
            circle = Circle(item, particle_radius, color='green')
            allArtists.append(ax.add_patch(circle))
    
    if len(cluster2) > 0:
        for clst in cluster2:
            clst = Circle(clst, particle_radius, color='blue')
            allArtists.append(ax.add_patch(circle))

    circle = Circle(tuple(delivery_station), particle_radius, facecolor='yellow', edgecolor='black')
    allArtists.append(ax.add_patch(circle))

    ax.axis('scaled')

    #ax.scatter(walls[:,0], walls[:,1], s=s, color='black')
        #ax.scatter(cluster2[:, 0], cluster2[:, 1], color='blue')
    ax.set_title("total delivered items: " + str(currently_collected))

    return allArtists

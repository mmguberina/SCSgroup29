import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection

def visualise(x, y, item_positions, N, nOfRobots, particle_radius, ax, camera, s, nOfCollectedItemsPerTime, item_positions_listPerTime, delivery_station, obstacles, obstacleRadius, torque_radius):
    item_positions_listPerTime.reverse()
    nOfCollectedItemsPerTime.reverse()
    currently_collected = 0

    for timestep in range(N):
       # for i in range(nOfRobots):
        if timestep % 40 == 0:

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
                # calculate the norm of every direction vector
                rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))

                cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfRobots) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})


            #cluster2 = np.array(list(cluster2))

            #ax.scatter(x[:, timestep], y[:, timestep], s=s, color='red')
            #ax.scatter(x[:, timestep], y[:, timestep], color='red')
            #ax.scatter(item_positions[:, 0], item_positions[:, 1], s=s, color='green')
            #ax.scatter(item_positions[:, 0], item_positions[:, 1], color='green')
            #ax.scatter(delivery_station[0], delivery_station[1], s=s, color='yellow')

            # paint blue if clustered
            #if len(cluster2 > 0):
            #    ax.scatter(cluster2[:, 0], cluster2[:, 1], s=s, color='blue')

            # do obstacles
            for obstacle in obstacles:
                circle = Circle(tuple(obstacle), obstacleRadius, color='black')
                ax.add_patch(circle)

            for robot in zip(x[:,timestep], y[:,timestep]):
                circle = Circle(robot, particle_radius, color='red')
                visSphere = Circle(robot, torque_radius, alpha=0.3, color='wheat')
                ax.add_patch(visSphere)
                ax.add_patch(circle)

            for item in zip(item_positions[:,0], item_positions[:,1]):
                circle = Circle(item, particle_radius, color='green')
                ax.add_patch(circle)
            
            if len(cluster2) > 0:
                for clst in cluster2:
                    clst = Circle(clst, particle_radius, color='blue')
                    ax.add_patch(circle)

            circle = Circle(tuple(delivery_station), particle_radius, facecolor='yellow', edgecolor='black')

            ax.axis('scaled')

            #ax.scatter(walls[:,0], walls[:,1], s=s, color='black')
                #ax.scatter(cluster2[:, 0], cluster2[:, 1], color='blue')
            ax.set_title("total delivered items: " + str(currently_collected))
            camera.snap()

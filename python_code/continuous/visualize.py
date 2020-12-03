import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera

def visualise(x, y, item_positions, N, nOfRobots, particle_radius, ax, camera, s, nOfCollectedItemsPerTime, delivery_station):

    for timestep in range(N):
       # for i in range(nOfRobots):
        if timestep % 40 == 0:

            clusteredP = set()
            cluster2 = set()
            actActNeig = {i:[] for i in range(nOfRobots)}

            pos = np.hstack((x[:, timestep].reshape((nOfRobots,1)), 
                             y[:, timestep].reshape((nOfRobots,1))))

            for i in range(nOfRobots):

                r = pos[i] - pos 
                # calculate the norm of every direction vector
                rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
                #actActNeig[i] = set([part for part in range(nOfRobots) if rnorms[part] < 4.1 * particle_radius])

                cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfRobots) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})


            cluster2 = np.array(list(cluster2))

            ax.scatter(x[:, timestep], y[:, timestep], s=s, color='red')
            ax.scatter(item_positions[:, 0], item_positions[:, 1], s=s, color='green')
            ax.scatter(delivery_station[0], delivery_station[1], s=s, color='yellow')
            if len(cluster2 > 0):
                ax.scatter(cluster2[:, 0], cluster2[:, 1], s=s, color='blue')
            ax.set_title("thus far delivered: " + str(nOfCollectedItemsPerTime[timestep]))
            camera.snap()

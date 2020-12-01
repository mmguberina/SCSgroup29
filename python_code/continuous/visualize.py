import numpy as np
import matplotlib.pyplot as plt
from celluloid import Camera

def visualise(x, y, item_positions, N, nOfParticles, particle_radius, ax, camera, s):

    for timestep in range(N):
       # for i in range(nOfParticles):
        if timestep % 40 == 0:

            clusteredP = set()
            cluster2 = set()
            actActNeig = {i:[] for i in range(nOfParticles)}

            pos = np.hstack((x[:, timestep].reshape((nOfParticles,1)), 
                             y[:, timestep].reshape((nOfParticles,1))))

            for i in range(nOfParticles):

                r = pos[i] - pos 
                # calculate the norm of every direction vector
                rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
                #actActNeig[i] = set([part for part in range(nOfParticles) if rnorms[part] < 4.1 * particle_radius])

                cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfParticles) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})


            cluster2 = np.array(list(cluster2))

            ax.scatter(x[:, timestep], y[:, timestep], s=s, color='red')
            ax.scatter(item_positions[:, 0], item_positions[:, 1], s=s, color='green')
            if len(cluster2 > 0):
                ax.scatter(cluster2[:, 0], cluster2[:, 1], s=s, color='blue')
            camera.snap()

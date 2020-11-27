import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib


def activeSwimmers(x, y, fi, item_positions, n, dt, T0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

    nOfItems = len(item_positions)
    actActNeig = {i:[] for i in range(nOfParticles)}
    for step in range(n):

        randFactors = (np.random.random((nOfParticles, 1)) - 0.5) * ni

        pos = np.hstack((x[:, step].reshape((nOfParticles,1)), 
                         y[:, step].reshape((nOfParticles,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
        v_hat = np.hstack((np.cos(fi[:, step]) .reshape((nOfParticles,1)), 
                            np.sin(fi[:, step]).reshape((nOfParticles,1))))
        # init torque
        torque = np.zeros((nOfParticles,1))
        torque_item = np.zeros((nOfParticles,1))

        # TODO
        # BUT FIRST TRY THE HW3 MODEL 
        # calculate some other torque model
        # for instance the attraction-repulsion like with charged particles
        # attracted towards the item, repeled from other robots


        # calculate torque for each particle (single torque depends on all other particles) 
        for i in range(1, nOfParticles):
            # calculate direction vector to every vector ( r_i,i is not a thing tho)
            r = pos[i] - pos 
            r_item = pos[i] - item_positions 
            # calculate the norm of every direction vector
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
            # collect only nearby ones
            actActNeig[i] = [part for part in range(nOfParticles) if rnorms[part] < 10 * particle_radius]
            # we need rhat
            r_hat  = r / rnorms
            r_item_hat  = r_item / rnorms_item
            r_hat[i] = np.zeros(2)
            # dot 'em. dot does not support axis thing so we do it like this 
            dots = np.sum(v_hat[i] * r_hat, axis=1).reshape((nOfParticles,1))
            dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItems,1))
            coefs = dots / rnorms**2 
            coefs_item = dots_item / rnorms_item**2 
            # try repelling them now
            coefs[i] = 0
            # crosses v_i with r_i and does so for all i
            crosses = np.cross(v_hat[i], r_hat).reshape((nOfParticles, 1))
            crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItems, 1))
            particle_torques = coefs * crosses
            particle_torques_item = coefs_item * crosses_item

            if rnorms[i] > torque_radius:
                particle_torques[i] = 0

            # make this work
            if rnorms_item[i] > torque_radius:
                particle_torques_item[i] = 0

            torque[i] = np.sum(particle_torques) 
            torque_item[i] = np.sum(particle_torques_item) 
#            print(torque)


        torque = T0 * torque
        torque_item = T0 * torque_item
        fi[:, step+1] = fi[:, step] + torque_item.reshape((nOfParticles,)) \
                            - torque.reshape((nOfParticles,)) \
                            + randFactors.reshape((nOfParticles,))
        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfParticles,1))), 
                            np.sin(fi[:, step+1].reshape((nOfParticles,1)))))
        x[:, step+1] = (x[:,step] +  v * v_hat[:,0]) % gridSize
        y[:, step+1] = (y[:,step] +  v * v_hat[:,1]) % gridSize

        pos = np.hstack((x[:, step+1].reshape((nOfParticles,1)), 
                         y[:, step+1].reshape((nOfParticles,1))))

        for p in range(nOfParticles):
            r = pos[p] - pos 
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
            rnorms[p] = 'Inf'
            r_hat  = r / rnorms

            for n in actActNeig[p]:
                if rnorms[n] < 2 * particle_radius:
                    overlap = 2 * particle_radius - rnorms[n]
                    moveVec = r_hat[n] * (overlap / 2)
#                    print(moveVec)
                    x[p, step+1] += moveVec[0]
                    y[p, step+1] += moveVec[1]
                    x[n, step+1] -= moveVec[0]
                    y[n, step+1] -= moveVec[1]

    return x, y




nOfParticles = 30
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
x = np.zeros((1 * nOfParticles,N+1))
x[:,0] = np.random.random(nOfParticles) * gridSize
y = np.zeros((1 * nOfParticles,N+1))
y[:,0] = np.random.random(nOfParticles) * gridSize
fi = np.zeros((1 * nOfParticles,N+1))
fi[:,0] = np.random.random(nOfParticles) * 2*np.pi
#x[:, 0] = 0.0


# 5 items
item_positions = np.fix(np.random.random((5, 2)) * gridSize)

x, y = activeSwimmers(x, y, fi, item_positions, N, dt, torque0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (3*(ax.get_window_extent().width  / (gridSize+1.) * 72./fig.dpi) ** 2)


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

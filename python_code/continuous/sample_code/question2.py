import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib


def activeSwimmers(x, y, fi, n, dt, T0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

    # have the exact same active brownian motions for the particles
    # then super-impose interacting torque effects on top of it

    # adjust coefs of active brownian motion to fit eqs
#    gaussCoefs = np.array([[np.sqrt(2*trans_dif_T *dt)],
#        np.sqrt([2*trans_dif_T * dt]),np.sqrt([2*rot_dif_T*dt])])

    for step in range(n):
        # generate n samples from a normal distribution for 3 coordinates (x,y,fi)
        #gaussRand_x = norm.rvs(size=(nOfParticles, 3), scale=1) * gaussCoefs

        # init rands for torque need (most likely not needed)
        randFactors = (np.random.random((nOfParticles, 1)) - 0.5) * ni

        # concatenate x and y so that each particles is a position pair
        pos = np.hstack((x[:, step].reshape((nOfParticles,1)), 
                         y[:, step].reshape((nOfParticles,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
        v_hat = np.hstack((np.cos(fi[:, step]) .reshape((nOfParticles,1)), 
                            np.sin(fi[:, step]).reshape((nOfParticles,1))))
        # init torque
        torque = np.zeros((nOfParticles,1))

        # calculate torque for each particle (single torque depends on all other particles) 
        for i in range(1, nOfParticles):
            # calculate direction vector to every vector ( r_i,i is not a thing tho)
            r = pos[i] - pos 
            # calculate the norm of every direction vector
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
            # we need rhat
            r_hat  = r / rnorms
            r_hat[i] = np.zeros(2)
            # dot 'em. dot does not support axis thing so we do it like this 
            dots = np.sum(v_hat[i] * r_hat, axis=1).reshape((nOfParticles,1))
            coefs = dots / rnorms**2 # check whether the shapes are ok here, it works if they are
            coefs[i] = 0
            # crosses v_i with r_i and does so for all i
            crosses = np.cross(v_hat[i], r_hat).reshape((nOfParticles, 1))
            particle_torques = coefs * crosses

            if rnorms[i] > torque_radius:
                particle_torques[i] = 0

            torque[i] = np.sum(particle_torques) 
#            print(torque)


        torque = T0 * torque
#        print(torque)
       # fi[:, step+1] = fi[:, step] + torque.reshape((nOfParticles,)) \
       #                     + randFactors.reshape((nOfParticles,))
        fi[:, step+1] = fi[:, step] + torque.reshape((nOfParticles,)) \
                            + randFactors.reshape((nOfParticles,))
        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfParticles,1))), 
                            np.sin(fi[:, step+1].reshape((nOfParticles,1)))))
        #x[:, step+1] = (x[:,step] + gaussRand_x[0] + dt * v * v_hat[:,0]) % gridSize
        #y[:, step+1] = (y[:,step] + gaussRand_x[1] + dt * v * v_hat[:,1]) % gridSize
        #x[:, step+1] = (x[:,step] + dt * v * v_hat[:,0]) % gridSize
        #y[:, step+1] = (y[:,step] + dt * v * v_hat[:,1]) % gridSize
        x[:, step+1] = (x[:,step] +  v * v_hat[:,0]) % gridSize
        y[:, step+1] = (y[:,step] +  v * v_hat[:,1]) % gridSize

        pos = np.hstack((x[:, step+1].reshape((nOfParticles,1)), 
                         y[:, step+1].reshape((nOfParticles,1))))

        # now you need to ensure that they do not overlap
        # what if i push it into another particle tho???????????????????
        for p in range(nOfParticles):
            r = pos[p] - pos 
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
            rnorms[p] = 'Inf'
            r_hat  = r / rnorms
            for n in range(nOfParticles):
                if rnorms[n] < 2 * particle_radius:
                    overlap = 2 * particle_radius - rnorms[n]
                    moveVec = r_hat[n] * (overlap / 2)
#                    print(moveVec)
                    x[p, step+1] += moveVec[0]
                    y[p, step+1] += moveVec[1]
                    x[n, step+1] -= moveVec[0]
                    y[n, step+1] -= moveVec[1]


    return x, y




nOfParticles = 100
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
N = 6000
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

x, y = activeSwimmers(x, y, fi, N, dt, torque0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (3*(ax.get_window_extent().width  / (gridSize+1.) * 72./fig.dpi) ** 2)


for timestep in range(N):
   # for i in range(nOfParticles):
    if timestep % 40 == 0:

        clusteredP = set()
        cluster2 = []
        cluster3 = []
        cluster4 = []
        cluster5 = []
        cluster6 = []
        cluster7 = []
        actActNeig = {i:[] for i in range(nOfParticles)}

        pos = np.hstack((x[:, timestep].reshape((nOfParticles,1)), 
                         y[:, timestep].reshape((nOfParticles,1))))

        for i in range(nOfParticles):

            r = pos[i] - pos 
            # calculate the norm of every direction vector
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
            actActNeig[i] = set([part for part in range(nOfParticles) if rnorms[part] < 4.1 * particle_radius])

            #cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfParticles) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})
            clusteredP = clusteredP.union({j for j in range(nOfParticles) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})




        ax.scatter(x[:, timestep], y[:, timestep], s=s, color='red')
#        cluster2 = np.array(list(cluster2))
        for j in range(nOfParticles):
            cluster = actActNeig[j].intersection(clusteredP)
            if len(cluster) == 2:
                cluster2.append([x[j, timestep], y[j,timestep]])
            if len(cluster) == 3:
                cluster3.append([x[j, timestep], y[j,timestep]])
            if len(cluster) == 4:
                cluster4.append([x[j, timestep], y[j,timestep]])
            if len(cluster) == 5:
                cluster5.append([x[j, timestep], y[j,timestep]])
            if len(cluster) == 6:
                cluster6.append([x[j, timestep], y[j,timestep]])
            if len(cluster) == 7:
                cluster7.append([x[j, timestep], y[j,timestep]])


        cluster2 =np.array(cluster2) 
        cluster3 =np.array(cluster3) 
        cluster4 =np.array(cluster4) 
        cluster5 =np.array(cluster5) 
        cluster6 =np.array(cluster6) 
        cluster7 =np.array(cluster7) 

        if len(cluster2) > 0:
            ax.scatter(cluster2[:, 0], cluster2[:, 1], s=s, color='blue')
        if len(cluster3) > 0:
            ax.scatter(cluster3[:, 0], cluster3[:, 1], s=s, color='green')
        if len(cluster4) > 0:
            ax.scatter(cluster4[:, 0], cluster4[:, 1], s=s, color='cyan')
        if len(cluster5) > 0:
            ax.scatter(cluster5[:, 0], cluster5[:, 1], s=s, color='pink')
        if len(cluster6) > 0:
            ax.scatter(cluster6[:, 0], cluster6[:, 1], s=s, color='gold')
        if len(cluster7) > 0:
            ax.scatter(cluster7[:, 0], cluster7[:, 1], s=s, color='ivory')
    #    for p in range(nOfParticles):

#        for p in range(nOfParticles):
            #ax.plot(x[p, -10:], y[p, -10:], color='blue')
#ax.plot(x[2, :], y[2, :], color='blue')
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
animation.save('question2colorclustwork_ni=' +str(ni)  +'.mp4')
plt.show()

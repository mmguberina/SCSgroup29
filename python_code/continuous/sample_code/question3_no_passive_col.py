import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib


def activeSwimmers(x, y, fi, xP, yP, n, dt, T0, nOfActiveParticles, nOfPassiveParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

    # we can do all random numbers for the passive particles here
    pasRandX = norm.rvs(size=(nOfPassiveParticles,) + (n,), scale=0.1*v)
    pasRandY = norm.rvs(size=(nOfPassiveParticles,) + (n,), scale=0.1*v)


    # have the exact same active brownian motions for the particles
    # then super-impose interacting torque effects on top of it

    # adjust coefs of active brownian motion to fit eqs
#    gaussCoefs = np.array([[np.sqrt(2*trans_dif_T *dt)],
#        np.sqrt([2*trans_dif_T * dt]),np.sqrt([2*rot_dif_T*dt])])

# basically now just repeat everything but for the passive particles

    for step in range(n):
        # generate n samples from a normal distribution for 3 coordinates (x,y,fi)
        #gaussRand_x = norm.rvs(size=(nOfParticles, 3), scale=1) * gaussCoefs

        # init rands for torque need (most likely not needed)
        randFactors = (np.random.random((nOfActiveParticles, 1)) - 0.5) * ni

        # concatenate x and y so that each particles is a position pair
        pos = np.hstack((x[:, step].reshape((nOfActiveParticles,1)), 
                         y[:, step].reshape((nOfActiveParticles,1))))

        posPas = np.hstack((xP[:, step].reshape((nOfPassiveParticles,1)), 
                         yP[:, step].reshape((nOfPassiveParticles,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
        v_hat = np.hstack((np.cos(fi[:, step]) .reshape((nOfActiveParticles,1)), 
                            np.sin(fi[:, step]).reshape((nOfActiveParticles,1))))
        # init torque
        torque = np.zeros((nOfActiveParticles,1))


        # calculate torque for each particle (single torque depends on all other particles) 
        for i in range(nOfActiveParticles):
            # calculate direction vector to every vector ( r_i,i is not a thing tho)
            r = pos[i] - pos 
            rPas = pos[i] - posPas
            # calculate the norm of every direction vector
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfActiveParticles,1))
            rnormsPas = np.linalg.norm(rPas, axis=1).reshape((nOfPassiveParticles,1))
            # we need rhat
            r_hat  = r / rnorms
            r_hat[i] = np.zeros(2)
            r_hatPas = rPas / rnormsPas
            # dot 'em. dot does not support axis thing so we do it like this 
            dots = np.sum(v_hat[i] * r_hat, axis=1).reshape((nOfActiveParticles,1))
            dotsPas = np.sum(v_hat[i] * r_hatPas, axis=1).reshape((nOfPassiveParticles,1))
            coefs = dots / rnorms**2 # check whether the shapes are ok here, it works if they are
            coefsPas = dotsPas / rnormsPas**2
            coefs[i] = 0
            # crosses v_i with r_i and does so for all i
            crosses = np.cross(v_hat[i], r_hat).reshape((nOfActiveParticles, 1))
            crossesPas = np.cross(v_hat[i], r_hatPas).reshape((nOfPassiveParticles, 1))
            particle_torques = coefs * crosses
            particle_torquesPas = (coefsPas * crossesPas).reshape((nOfPassiveParticles,))

            if rnorms[i] > torque_radius:
                particle_torques[i] = 0
            
            particle_torquesPas = np.array([0 if rnormsPas[pas] < torque_radius else particle_torquesPas[pas]
                    for pas in range(nOfPassiveParticles)])

            torque[i] = np.sum(particle_torques) - np.sum(particle_torquesPas)
#            print(torque)


        torque = T0 * torque

        #doPassive
        xP[:, step+1] = (xP[:,step] + v * pasRandX[:,step]) % gridSize
        yP[:, step+1] = (yP[:,step] + v * pasRandY[:,step]) % gridSize

#        print(torque)
       # fi[:, step+1] = fi[:, step] + torque.reshape((nOfParticles,)) \
       #                     + randFactors.reshape((nOfParticles,))
        fi[:, step+1] = fi[:, step] + torque.reshape((nOfActiveParticles,)) \
                            + randFactors.reshape((nOfActiveParticles,))
        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfActiveParticles,1))), 
                            np.sin(fi[:, step+1].reshape((nOfActiveParticles,1)))))
        #x[:, step+1] = (x[:,step] + gaussRand_x[0] + dt * v * v_hat[:,0]) % gridSize
        #y[:, step+1] = (y[:,step] + gaussRand_x[1] + dt * v * v_hat[:,1]) % gridSize
        #x[:, step+1] = (x[:,step] + dt * v * v_hat[:,0]) % gridSize
        #y[:, step+1] = (y[:,step] + dt * v * v_hat[:,1]) % gridSize
        x[:, step+1] = (x[:,step] +  v * v_hat[:,0]) % gridSize
        y[:, step+1] = (y[:,step] +  v * v_hat[:,1]) % gridSize

        pos = np.hstack((x[:, step+1].reshape((nOfActiveParticles,1)), 
                         y[:, step+1].reshape((nOfActiveParticles,1))))

        posPas = np.hstack((xP[:, step].reshape((nOfPassiveParticles,1)), 
                         yP[:, step].reshape((nOfPassiveParticles,1))))
        # now you need to ensure that they do not overlap
        # what if i push it into another particle tho???????????????????
        for p in range(nOfActiveParticles):
            r = pos[p] - pos 
            rPas = pos[i] - posPas
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfActiveParticles,1))
            rnormsPas = np.linalg.norm(rPas, axis=1).reshape((nOfPassiveParticles,1))
            rnorms[p] = 'Inf'
            r_hat  = r / rnorms
            r_hatPas = rPas / rnormsPas
            for n in range(nOfActiveParticles):
                if rnorms[n] < 2 * particle_radius:
                    overlap = 2 * particle_radius - rnorms[n]
                    moveVec = r_hat[n] * (overlap / 2)
#                    print(moveVec)
                    x[p, step+1] += moveVec[0]
                    y[p, step+1] += moveVec[1]
                    x[n, step+1] -= moveVec[0]
                    y[n, step+1] -= moveVec[1]

            for n in range(nOfPassiveParticles):
                if rnormsPas[n] < 2 * particle_radius:
                    overlap = 2 * particle_radius - rnormsPas[n]
                    moveVec = r_hatPas[n] * (overlap / 2)
#                    print(moveVec)
                    x[p, step+1] += moveVec[0]
                    y[p, step+1] += moveVec[1]
                    xP[n, step+1] -= moveVec[0]
                    yP[n, step+1] -= moveVec[1]

    return x, y, xP, yP




nOfActiveParticles = 20
nOfPassiveParticles = 200
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
#ni = 0.0002
ni = 0.002
v = 0.05
# Total time.
T = 50
#gridSize = 100
gridSize = 10
torque0 = 1
particle_radius = 0.2
torque_radius = 3 * particle_radius

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 2000
# Time step size
dt = T/N
# Initial values of x.
# you should init this to sth other than 0
x = np.zeros((1 * nOfActiveParticles,N+1))
x[:,0] = np.random.random(nOfActiveParticles) * gridSize
y = np.zeros((1 * nOfActiveParticles,N+1))
y[:,0] = np.random.random(nOfActiveParticles) * gridSize
fi = np.zeros((1 * nOfActiveParticles,N+1))
fi[:,0] = np.random.random(nOfActiveParticles) * 2*np.pi
#x[:, 0] = 0.0


xP = np.zeros((1 * nOfPassiveParticles,N+1))
xP[:,0] = np.random.random(nOfPassiveParticles) * gridSize
yP = np.zeros((1 * nOfPassiveParticles,N+1))
yP[:,0] = np.random.random(nOfPassiveParticles) * gridSize

x, y, xP, yP = activeSwimmers(x, y, fi, xP, yP, N, dt, torque0, nOfActiveParticles, nOfPassiveParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
#s = (3*(ax.get_window_extent().width  / (gridSize+1.) * 72./fig.dpi) ** 2)


for timestep in range(N):
   # for i in range(nOfParticles):
    if timestep % 15 == 0:

        cluster2 = set()
        cluster3 = []
        cluster4 = []
        cluster5 = []
        cluster6 = []
        cluster7 = []

        pos = np.hstack((x[:, timestep].reshape((nOfActiveParticles,1)), 
                         y[:, timestep].reshape((nOfActiveParticles,1))))

        for i in range(nOfActiveParticles):

            r = pos[i] - pos 
            # calculate the norm of every direction vector
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfActiveParticles,1))

            cluster2 = cluster2.union({tuple(pos[j]) for j in range(nOfActiveParticles) if rnorms[j] < 2.1*particle_radius and rnorms[j] > 0})




#        ax.scatter(x[:, timestep], y[:, timestep], s=s, color='red')
        ax.scatter(x[:, timestep], y[:, timestep], color='red')
#        ax.scatter(xP[:, timestep], yP[:,timestep], s=s, color='gray')
        ax.scatter(xP[:, timestep], yP[:,timestep], color='gray')
        cluster2 = np.array(list(cluster2))
        if len(cluster2) > 0:
#            ax.scatter(cluster2[:, 0], cluster2[:, 1], s=s, color='blue')
            ax.scatter(cluster2[:, 0], cluster2[:, 1], color='blue')
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
animation.save('question3_=' + str(ni) +'.mp4')
plt.show()

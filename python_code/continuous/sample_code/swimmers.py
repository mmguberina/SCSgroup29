import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera


def activeSwimmers(x, n, dt, T0, nOfParticles, ni, v, gridSize, particle_radius, torque_radius, out=None):

    # one step is:
    # x_n(t+1) = x_n(t) + v_n(t+1)
    # fi_n(t+1) = fi_n(t) + T_n + ksi # |v| costs, so you got
    # v_x = cos(fi), v_y = sin(fi)
    # T_n is the torque which you compute given the other particles
    # ksi is just a sample from uniform from  (-ni/2, ni/2)

    for step in range(n):
        # init rands you'll need
        randFactors = (np.random.random((nOfParticles, 1)) - 0.5) * ni

        # concatenate x and y so that each particles is a position pair
        pos = np.hstack((x[:, step].reshape((nOfParticles,1)), 
                         y[:, step].reshape((nOfParticles,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
        v_hat = np.hstack((np.cos(fi[:, step].reshape((nOfParticles,1))), 
                            np.sin(fi[:, step].reshape((nOfParticles,1)))))
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
            dots = np.sum(r_hat * v_hat, axis=1).reshape((nOfParticles,1))
            coefs = dots / rnorms**2 # check whether the shapes are ok here, it works if they are
            coefs[i] = 0
            # crosses v_i with r_i and does so for all i
            crosses = np.cross(v_hat, r_hat).reshape((nOfParticles, 1))
            particle_torques = coefs * crosses

            if rnorms[i] > 0.25:
                particle_torques[i] = 0
            torque[i] = np.sum(particle_torques) 
#            print(torque)


        torque = T0 * torque
        fi[:, step+1] = fi[:, step] + torque.reshape((nOfParticles,)) \
                            + randFactors.reshape((nOfParticles,))
        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfParticles,1))), 
                            np.sin(fi[:, step+1].reshape((nOfParticles,1)))))
        x[:, step+1] = (x[:,step] + dt * v_hat[:,0]) % gridSize
        y[:, step+1] = (y[:,step] + dt * v_hat[:,1]) % gridSize

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
                    moveVec = r_hat[n] * overlap / 2
#                    print(moveVec)
                    x[p, step+1] -= moveVec[0]
                    y[p, step+1] -= moveVec[1]
                    x[n, step+1] += moveVec[0]
                    y[n, step+1] += moveVec[1]


    return x, y




nOfParticles = 100
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
ni = 0.1
v = 2
# Total time.
T = 10
gridSize = 2
torque0 = 1
particle_radius = 0.03
torque_radius = 4 * particle_radius
# Number of steps.
N = 2000
# Time step size
dt = T/N
# Initial values of x.
# you should init this to sth other than 0
x = np.zeros((1 * nOfParticles,N+1))
x[:,0] = np.random.random(nOfParticles)
y = np.zeros((1 * nOfParticles,N+1))
y[:,0] = np.random.random(nOfParticles)
fi = np.zeros((1 * nOfParticles,N+1))
fi[:,0] = np.random.random(nOfParticles)
#x[:, 0] = 0.0

x, y = activeSwimmers(x, N, dt, torque0, nOfParticles, ni, v, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
for timestep in range(N):
#    for i in range(nOfParticles):
    if timestep % 1 == 0:
        ax.scatter(x[:, timestep], y[:, timestep], color='red')
        camera.snap()

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
plt.show()

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib


def activeSwimmers(x, y, fi, n, dt, T0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

#    fi[:,0] = np.array([-np.pi/2, np.pi/2])
    fi[:,0] = np.array([0, 0, np.pi/2])

    for step in range(n):
        pos = np.hstack((x[:, step].reshape((nOfParticles,1)), 
                         y[:, step].reshape((nOfParticles,1))))
        # also get the speeds for each particle
        # this already is vhat 'cos sin^2(x) + cos^2(x) = 1
        v_hat = np.hstack((np.cos(fi[:, step]).reshape((nOfParticles,1)), 
                            np.sin(fi[:, step]).reshape((nOfParticles,1))))

        torque = np.zeros((nOfParticles,1))
        for i in range(nOfParticles):
            # calculate direction vector to every vector ( r_i,i is not a thing tho)
            r = pos[i] - pos 
#            r = pos - pos[i]
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
#            print(i, particle_torques)

            if rnorms[i] > torque_radius:
                particle_torques[i] = 0

            torque[i] = np.sum(particle_torques) 
        
        torque = T0 * torque
#        print(torque)

        fi[:, step+1] = fi[:, step] + torque.reshape((nOfParticles,))
        v_hat = np.hstack((np.cos(fi[:, step+1].reshape((nOfParticles,1))), 
                            np.sin(fi[:, step+1].reshape((nOfParticles,1)))))
        x[:, step+1] = (x[:,step] +  v * v_hat[:,0]) % gridSize
        y[:, step+1] = (y[:,step] +  v * v_hat[:,1]) % gridSize

        pos = np.hstack((x[:, step+1].reshape((nOfParticles,1)), 
                         y[:, step+1].reshape((nOfParticles,1))))

        # now you need to ensure that they do not overlap
        # what if i push it into another particle tho???????????????????
        #print(pos)
        for p in range(nOfParticles):
            r = pos[p] - pos 
#            print(r)
            rnorms = np.linalg.norm(r, axis=1).reshape((nOfParticles,1))
            rnorms[p] = 'Inf'
            r_hat  = r / rnorms
#            print(rnorms)
            for n in range(nOfParticles):
                if rnorms[n] < 2 * particle_radius:

                    overlap = 2 * particle_radius - rnorms[n]
                    moveVec = r_hat[n] * (overlap / 2)
                    x[p, step+1] += moveVec[0]
                    y[p, step+1] += moveVec[1]
                    x[n, step+1] -= moveVec[0]
                    y[n, step+1] -= moveVec[1]


    return x, y, fi




nOfParticles = 3
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
ni = 0.002
v = 0.05
# Total time.
T = 1
gridSize = 100
torque0 = 1
particle_radius = 1
torque_radius = 6 * particle_radius

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 400
# Time step size
dt = T/N
# Initial values of x.
# you should init this to sth other than 0
x = np.zeros((1 * nOfParticles,N+1))
y = np.zeros((1 * nOfParticles,N+1))
fi = np.zeros((1 * nOfParticles,N+1))


#x[0] = np.array([49.])
#y[0] = np.array([51])
#x[1] = np.array([51.])
#y[1] = np.array([49])

x[0] = np.array([49.])
y[0] = np.array([52])
x[1] = np.array([49.])
y[1] = np.array([49])
x[2] = np.array([52.])
y[2] = np.array([49])

x, y, fi = activeSwimmers(x, y, fi, N, dt, torque0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.set_xlim(45,55)
ax.set_ylim(45,55)
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (10000)

#ax.scatter(x[0, 0], y[0, 0], s=s, color='red')
for timestep in range(N):
   # for i in range(nOfParticles):
#    if timestep == 100 or timestep == 1001:
    ax.scatter(x[0, timestep], y[0, timestep], s=s, color='red')
    ax.scatter(x[1, timestep], y[1, timestep], s=s, color='blue')
    ax.scatter(x[2, timestep], y[2, timestep], s=s, color='green')
#    ax.quiver(x[0, timestep], y[0, timestep], np.cos(fi[0, timestep]), np.sin(fi[0,timestep]))
#    ax.quiver(x[1, timestep], y[1, timestep], np.cos(fi[1, timestep]), np.sin(fi[1,timestep]))

#    print(x[:, timestep], y[:, timestep])
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
animation.save('somerun.mp4')
plt.show()

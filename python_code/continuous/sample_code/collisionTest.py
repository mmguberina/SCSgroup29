import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera
import matplotlib


def activeSwimmers(x, y, fi, n, dt, T0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None):

    fi[:,0] = np.array([0, np.pi])
    for step in range(n):

        fi[:, step+1] = fi[:, step] 
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
                    print("particle_radius")
                    print(particle_radius)
                    print("rnorms[n]")
                    print(rnorms[n])
                    print("overlap")
                    print(overlap)
                    print("moveVec")
                    print(moveVec)
#                    print(moveVec)
                    x[p, step+1] += moveVec[0]
                    y[p, step+1] += moveVec[1]
                    x[n, step+1] -= moveVec[0]
                    y[n, step+1] -= moveVec[1]


    return x, y




nOfParticles = 2
#rot_dif_T = 0.2
#trans_dif_T = 0.2
#v = 1
ni = 0.002
v = 0.05
# Total time.
T = 0.1
gridSize = 100
torque0 = 10
particle_radius = 0.5
torque_radius = 4 * particle_radius

rot_dif_T = 0.2
trans_dif_T = 0.2
# Number of steps.
N = 100
# Time step size
dt = T/N
# Initial values of x.
# you should init this to sth other than 0
x = np.zeros((1 * nOfParticles,N+1))
x[0] = np.array([48])
x[1] = np.array([53])
y = np.zeros((1 * nOfParticles,N+1))
y[0] = np.array([50])
y[1] = np.array([50])
fi = np.zeros((1 * nOfParticles,N+1))

x, y = activeSwimmers(x, y, fi, N, dt, torque0, nOfParticles, ni, v, trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius)

fig, ax = plt.subplots()
ax.grid()
# Plot the 2D trajectory.
camera = Camera(fig)
s = (9000)
for timestep in range(N):
   # for i in range(nOfParticles):
#    if timestep == 100 or timestep == 1001:
    ax.scatter(x[0, timestep], y[0, timestep], s=s, color='red')
    ax.scatter(x[1, timestep], y[1, timestep], s=s, color='blue')
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

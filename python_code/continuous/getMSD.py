import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from celluloid import Camera

def swimstep(x0, n, dt, v, trans_dif_T, rot_dif_T, out=None):
    # generate n samples from a normal distribution for 3 coordinates
    r = norm.rvs(size=x0.shape + (n,), scale=1)

    # multiply these random numbers with their appropriate coeficients
    mulcoefs = np.array([[np.sqrt(2*trans_dif_T *dt)],
        np.sqrt([2*trans_dif_T * dt]),np.sqrt([2*rot_dif_T*dt])])
    r = mulcoefs * r

    # and now x and y coordinates the angles part
    # this works because the angles are independent and can be integrated separately
    fis = np.cumsum(r[2])
    r[0] += v*dt*np.cos(fis / v)
    r[1] += v*dt*np.sin(fis / v)
   
    # initialize the output array
    if out is None:
        out = np.empty(r.shape)

    # generate brownian motion as the cumulative sum
    np.cumsum(r, axis=-1, out=out)

    # just append the initial condition and stick it to out
    out += np.expand_dims(x0, axis=-1)
    return out




def MSD(X, dt, T):
    rez = [(X[0][i] - X[0][0])**2 + (X[1][i] - X[1][0])**2 
            for i in range(int(T//dt))]
    return np.asarray(rez)

def calcMSDAvg(rot_dif_T, trans_dif_T, v, T, N, ax):
#    t = np.linspace(0,T,int(T//dt))
    msd_sum = np.zeros(int(T//dt))
    nOfIt = 100
    for i in range(nOfIt):
        swimstep(x[:,0], N, dt, v, trans_dif_T, rot_dif_T, out=x[:,1:])

        msd = MSD(x, dt, T)
        msd_sum += msd
    ax.plot(x[0,:], x[1,:], label="v = " + str(v))

    return msd_sum / nOfIt


nOfParticles = 1
rot_dif_T = 0.2
trans_dif_T = 0.2
v = 0.001
# Total time.
T = 50.0
# Number of steps.
N = 2000
# Time step size
dt = T/N
# Initial values of x.
x = np.zeros((3 * nOfParticles,N+1))
print(x)
#x[:, 0] = 0.0

#swimstep(x[:,0], N, dt, v, trans_dif_T, rot_dif_T, out=x[:,1:])

fig, ax = plt.subplots()
fig2, ax2 = plt.subplots()
#camera = Camera(fig)
# Plot the 2D trajectory.
#ax.plot(x[0,0],x[1,0], 'go')
#ax.plot(x[0,-1], x[1,-1], 'ro')
#for i in range(N):
#    ax.scatter(x[0][i],x[1][i])
#    camera.snap()

#ax.plot(x[0,:], x[1,:])
#animation = camera.animate()
#fig.show()

# Mark the start and end points.
#
## More plot decorations.
#ax.set_title('2D Brownian Motion')
#ax.set_xlabel('x', fontsize=16)
#ax.set_ylabel('y', fontsize=16)
#ax.axis('equal')
t = np.linspace(0,T,int(T//dt))
#msd = MSD(x, dt, T)
v1 =  0.001
v2 =  0.01
v3 =  0.1 
v4 =  1.0 
msd1 = calcMSDAvg(rot_dif_T, trans_dif_T, v1 , T, N, ax)
msd2 = calcMSDAvg(rot_dif_T, trans_dif_T, v2,  T, N, ax)
msd3 = calcMSDAvg(rot_dif_T, trans_dif_T, v3, T, N , ax)
msd4 = calcMSDAvg(rot_dif_T, trans_dif_T, v4, T, N , ax)
ax2.loglog(t, msd1, label="v = " + str(v1))
ax2.loglog(t, msd2, label="v = " + str(v2))
ax2.loglog(t, msd3, label="v = " + str(v3))
ax2.loglog(t, msd4, label="v = " + str(v4))
ax2.grid()
ax2.legend()
ax.grid()
ax.legend()
plt.savefig("question1_msd.png")
plt.show()

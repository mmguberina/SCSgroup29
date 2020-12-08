import numpy as np

#TODO
def calcObstacleField(obstacles, obstacleRadius, torque_radius):

    obs_forces = np.array([r_obs_hat[p] / (rnorms_obs[p] - obstacleRadius)**2 for p in range(nOfObssInNeigh)])
    force =  np.sum(obs_forces) 
    force_obs[i] = force if np.abs(force) < v else v * np.sign(force)

    return force_rob, force_item, force_obs

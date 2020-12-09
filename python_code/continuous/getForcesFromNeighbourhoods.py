import numpy as np

def calcTorqueRob(pos, robot_states, robRobNeig, v_hat, nOfRobots):
    torque_rob = np.zeros(nOfRobots)

    for i in range(nOfRobots):
        # if robot state is != 0 then there is no effect
        if robot_states[i] != 0:
            torque_rob[i] = 0
            continue

        nOfRobotsInNeigh = len(robRobNeig[i])

        if nOfRobotsInNeigh == 0:
            torque_rob[i] = 0
        else:
            pos_neighbs = np.array([pos[n] for n in robRobNeig[i]]) if nOfRobotsInNeigh > 0 else 0
            r_rob = pos[i] - pos_neighbs
            rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobotsInNeigh,1))
            r_rob_hat  = r_rob / rnorms_rob
            dots_rob = np.sum(v_hat[i] * r_rob_hat, axis=1).reshape((nOfRobotsInNeigh,1))
            coefs = dots_rob / rnorms_rob**2 
            crosses = np.cross(v_hat[i], r_rob_hat).reshape((nOfRobotsInNeigh, 1))
            particle_torques = coefs * crosses
            torque_rob[i] = np.sum(particle_torques) 

    return torque_rob



def calcTorqueItem(pos, robot_states, robItemNeig, v_hat, nOfRobots, nOfItems):

    torque_item = np.zeros(nOfRobots)
    # calculate torque for each particle based on nbhd
    for i in range(nOfRobots):
        # if robot state is != 0 then there is no effect
        if robot_states[i] != 0:
            torque_item[i] = 0
            continue

        nOfItemsInNeigh = len(robItemNeig[i])

        if nOfItemsInNeigh == 0:
            torque_item[i] = 0
        else:
            # calculate direction vector to things in neighbourhood
            r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
            # calculate the norm of every direction vector
            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
            # collect only nearby ones
            # we need rhat
            r_item_hat  = r_item / rnorms_item
            # dot 'em. dot does not support axis thing so we do it like this 
            dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItemsInNeigh,1))
            coefs_item = dots_item / rnorms_item**2 
            # try repelling them now
            # crosses v_i with r_i and does so for all i
            crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItemsInNeigh, 1))
            particle_torques_item = coefs_item * crosses_item

            torque_item[i] = np.sum(particle_torques_item) 

    return torque_item



def calcTorqueObs(pos, robot_states, robObsNeig, v_hat, nOfRobots, obstacleRadius):

    torque_obs = np.zeros(nOfRobots)
    # calculate torque for each particle based on nbhd
    for i in range(nOfRobots):
        # if robot state is != 0 then there is no effect
        nOfObssInNeigh = len(robObsNeig[i])

        if nOfObssInNeigh == 0:
            torque_obs[i] = 0
            continue
        # calculate direction vector to things in neighbourhood
        r_obs = pos[i] - np.array(list(map(list, robObsNeig[i])))
        # calculate the norm of every direction vector
        # the robot will never get inside the obstacle so minus it to have reasonable forces
        rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1)) - obstacleRadius
        # collect only nearby ones
        # we need rhat
        r_obs_hat  = r_obs / rnorms_obs
        # dot 'em. dot does not support axis thing so we do it like this 
        dots_obs = np.sum(v_hat[i] * r_obs_hat, axis=1).reshape((nOfObssInNeigh,1))
        coefs_obs = dots_obs / rnorms_obs**2 
        # try repelling them now
        # crosses v_i with r_i and does so for all i
        crosses_obs = np.cross(v_hat[i], r_obs_hat).reshape((nOfObssInNeigh, 1))
        particle_torques_obs = coefs_obs * crosses_obs

        torque_obs[i] = np.sum(particle_torques_obs) 

    return torque_obs




def calcForceRob(v, pos, robot_states, robRobNeig, v_hat, nOfRobots, particle_radius):

    force_rob = np.zeros((nOfRobots,2))
    for i in range(nOfRobots):
        nOfRobotsInNeigh = len(robRobNeig[i])

        if nOfRobotsInNeigh == 0:
            force_rob[i] = np.zeros(2)
        else:
            pos_neighbs = np.array([pos[n] for n in robRobNeig[i]]) if nOfRobotsInNeigh > 0 else 0
            r_rob = pos[i] - pos_neighbs
            rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobotsInNeigh,1))
            r_rob_hat  = r_rob / rnorms_rob
            #rob_forces = np.array([r_rob_hat[p] / (rnorms_rob[p] - particle_radius)**2 for p in range(nOfRobotsInNeigh)])
            # trying it without the square 'cos it seemed way too small
            rob_forces = np.array([r_rob_hat[p] / (rnorms_rob[p] - particle_radius) for p in range(nOfRobotsInNeigh)])
            force =  np.sum(rob_forces) 
            # it works because it's per element
            force_rob[i] = force if np.abs(force) < v else 0.05 * np.sign(force)

    return force_rob




def calcForceItem(v, pos, robot_states, robItemNeig, v_hat, nOfRobots, nOfItems, particle_radius):

    force_item = np.zeros((nOfRobots,2))
    for i in range(nOfRobots):
        nOfItemsInNeigh = len(robItemNeig[i])
        if nOfItemsInNeigh == 0:
            force_item[i] = np.zeros(2)
        else:
            r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
            r_item_hat  = r_item / rnorms_item
            item_forces = np.array([r_item_hat[p] / (rnorms_item[p])**2 for p in range(nOfItemsInNeigh)])
            force =  np.sum(item_forces) 
            force_item[i] = force if np.abs(force) < v else v * np.sign(force)

    return force_item




def calcForceObs(v, pos, robot_states, robObsNeig, v_hat, nOfRobots, nOfItems, particle_radius, obstacleRadius):

    force_obs = np.zeros((nOfRobots,2))
    for i in range(nOfRobots):
        nOfObssInNeigh = len(robObsNeig[i])
        if nOfObssInNeigh == 0:
            force_obs[i] = np.zeros(2)
        else:
            r_obs = pos[i] - np.array(list(map(list, robObsNeig[i])))
            rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1))
            r_obs_hat  = r_obs / rnorms_obs
            obs_forces = np.array([r_obs_hat[p] / (rnorms_obs[p] - obstacleRadius)**2 for p in range(nOfObssInNeigh)])
            force =  np.sum(obs_forces) 
            force_obs[i] = force if np.abs(force) < v else v * np.sign(force)

    return force_obs





import numpy as np

# TODO
# this needs to be changed so that obss block vision, i.e. remove items which they are in front of
# this will involve doing some geometry
# or just make obstacles big lel hack
def getNeighbourhoods(pos, item_positions_list, nOfRobots, nOfItems, particle_radius, torque_radius, obstacles, obstacleRadius):
    nOfObss = len(obstacles)
    robRobNeig = {i:[] for i in range(nOfRobots)}
    robItemNeig = {i:[] for i in range(nOfRobots)}
    robObsNeig = {i:[] for i in range(nOfRobots)}
    for i in range(nOfRobots):
        # calculate direction vector to every vector ( r_i,i is not a thing tho)
        r_rob = pos[i] - pos 
        r_item = pos[i] - item_positions_list
        r_obs  = pos[i] - obstacles
        # calculate the norm of every direction vector
        rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobots,1))
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
        rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObss,1))

        # form neighbourhoods 
        robRobNeig[i] = {rob for rob in range(nOfRobots) 
                if rnorms_rob[rob] < torque_radius and rob != i}
        robItemNeig[i] = {tuple(item_positions_list[it]) for it in range(nOfItems) if rnorms_item[it] < torque_radius}
        robObsNeig[i] = {tuple(obstacles[o]) for o in range(nOfObss) 
                if rnorms_obs[o] - obstacleRadius < torque_radius}
    return robRobNeig, robItemNeig, robObsNeig




def calcTorqueFromNeigh(pos, robot_states, robRobNeig, robItemNeig, v_hat, nOfRobots, nOfItems):

    torque_rob = np.zeros((nOfRobots,1))
    torque_item = np.zeros((nOfRobots,1))
    # calculate torque for each particle based on nbhd
    for i in range(nOfRobots):

        # if robot state is != 0 then there is no effect
        if robot_states[i] != 0:
            torque_rob[i] = 0
            torque_item[i] = 0
            continue

        nOfRobotsInNeigh = len(robRobNeig[i])
        nOfItemsInNeigh = len(robItemNeig[i])

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


    return torque_rob, torque_item



def calcForceAttractionRepulsion(v, pos, robot_states, robRobNeig, robItemNeig, robObsNeig, v_hat, nOfRobots, nOfItems, obstacleRadius):

    force_rob = np.zeros((nOfRobots,2))
    force_item = np.zeros((nOfRobots,2))
    force_obs = np.zeros((nOfRobots,2))
    # calculate torque for each particle based on nbhd 
    for i in range(nOfRobots):

        nOfRobotsInNeigh = len(robRobNeig[i])
        nOfItemsInNeigh = len(robItemNeig[i])
        nOfObssInNeigh = len(robObsNeig[i])

        if nOfRobotsInNeigh == 0:
            force_rob[i] = np.zeros(2)
        else:
            pos_neighbs = np.array([pos[n] for n in robRobNeig[i]]) if nOfRobotsInNeigh > 0 else 0
            r_rob = pos[i] - pos_neighbs
            rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobotsInNeigh,1))
            r_rob_hat  = r_rob / rnorms_rob
            rob_forces = np.array([r_rob_hat[p] / (rnorms_rob[p])**2 for p in range(nOfRobotsInNeigh)])
            force =  np.sum(rob_forces) 
            force_rob[i] = force if np.abs(force) < v else 0.05 * np.sign(force)


        if nOfItemsInNeigh == 0:
            force_item[i] = np.zeros(2)
        else:
            r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
            r_item_hat  = r_item / rnorms_item
            item_forces = np.array([r_item_hat[p] / (rnorms_item[p])**2 for p in range(nOfItemsInNeigh)])
            force =  np.sum(item_forces) 
            force_item[i] = force if np.abs(force) < v else v * np.sign(force)


        if nOfObssInNeigh == 0:
            force_obs[i] = np.zeros(2)
        else:
            r_obs = pos[i] - np.array(list(map(list, robObsNeig[i])))
            rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1))
            r_obs_hat  = r_obs / rnorms_obs
            obs_forces = np.array([r_obs_hat[p] / (rnorms_obs[p] - obstacleRadius)**2 for p in range(nOfObssInNeigh)])
            force =  np.sum(obs_forces) 
            force_obs[i] = force if np.abs(force) < v else v * np.sign(force)

    return force_rob, force_item, force_obs







# return list of robots that are at the delivery station too
# ==> just return 0,0 if they are
def v_hat2DeliveryStation(pos, delivery_station, particle_radius):
    rToDelivery = delivery_station - pos
    rnorms = np.linalg.norm(rToDelivery, axis=1)
    isDone = rnorms < particle_radius
    v_hat2DeliveryStation = rToDelivery / rnorms.reshape((len(rToDelivery),1))

    v_hat2DeliveryStation = np.array([ [0,0] if isDone[i] \
                                else v_hat2DeliveryStation[i]for i in range(len(rnorms))])
    return v_hat2DeliveryStation


# makes code cleaner, but i don't want these huge x's and y's sloshing around
# test whether just references or whole objects are passed (most likely they are refs but check)
def volumeExclusion(x, y, pos, step, robRobNeig, robObsNeig, particle_radius, nOfRobots, obstacleRadius):

    for p in range(nOfRobots):
        if len(robRobNeig[p]) == 0:
            continue
        r = pos[p] - pos 
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
        rnorms[p] = 'Inf'
        r_hat  = r / rnorms
        for n in robRobNeig[p]:
            if rnorms[n] < 2 * particle_radius:
                overlap = 2 * particle_radius - rnorms[n]
                moveVec = r_hat[n] * (overlap / 2)
                x[p, step+1] += moveVec[0]
                y[p, step+1] += moveVec[1]
                x[n, step+1] -= moveVec[0]
                y[n, step+1] -= moveVec[1]


    # you can change the radius of barriers if you feel like it
    for p in range(nOfRobots):
        if len(robObsNeig[p]) == 0:
            continue
        robObsNeigList = np.array(list(robObsNeig[p]))
        r = pos[p] - robObsNeigList
        rnorms = np.linalg.norm(r, axis=1).reshape((len(robObsNeigList),1))
        r_hat  = r / rnorms
        for n in range(len(robObsNeigList)):
            if rnorms[n] < particle_radius + obstacleRadius:
                overlap = particle_radius + obstacleRadius - rnorms[n]
                moveVec = r_hat[n] * (overlap)
                x[p, step+1] += moveVec[0]
                y[p, step+1] += moveVec[1]


def handleItems(x, y, step, robItemNeig, pos, robot_storage, robot_states, item_positions_set, item_positions_listPerTime, particle_radius, nOfRobots, nOfItems):
    for p in range(nOfRobots):
        for n in robItemNeig[p]:
            rnorm_item = np.linalg.norm(np.array(n) - pos[p])
            if rnorm_item < 2 * particle_radius:
                item_positions_set.remove(n)
                nOfItems -= 1
                robot_storage[p].append(n)
                robot_states[p] = 1

                item_positions_list = np.array(list(item_positions_set))
                item_positions_listPerTime.append([step, item_positions_list])
    return np.array(list(item_positions_set)), nOfItems

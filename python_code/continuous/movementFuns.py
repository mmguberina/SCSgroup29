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



def calcTorqueObs(pos, robot_states, robObsNeig, v_hat, nOfRobots):

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
        rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObssInNeigh,1))
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
            rob_forces = np.array([r_rob_hat[p] / (rnorms_rob[p] - particle_radius)**2 for p in range(nOfRobotsInNeigh)])
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




# go to delivery station if you're in state 1
def v_hat2DeliveryStationFromState(pos, delivery_station, particle_radius, robot_states, nOfRobots):
    v_hat2DeliveryStation = {}
    for i in range(nOfRobots):
        if robot_states[i] != 1:
            continue
        rToDelivery = delivery_station - pos[i]
        rnorms = np.linalg.norm(rToDelivery)
        isDone = rnorms < particle_radius
        v_hat = rToDelivery / rnorms
        
        # zeros means it's done
        v_hat2DeliveryStation[i] = np.zeros(2) if isDone else v_hat
    return v_hat2DeliveryStation




# go to the closest item if you're in state 2
def v_hat2NearItem(pos, robItemNeig, particle_radius, nOfRobots, robot_states):
    robsWithNearItems = {}
    for i in range(nOfRobots):
        nOfItemsInNeigh = len(robItemNeig[i])
        # this does not interest you if you're delivering
        if robot_states[i] == 1:
            continue
        if  nOfItemsInNeigh == 0:
            continue
        nearItemsList = np.array(list(map(list, robItemNeig[i])))
        r_item = nearItemsList - pos[i] 
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,))
        # sort by index to get the one you want
        closest = np.argsort(rnorms_item)[0]
        r_closest_hat  = r_item[closest] / rnorms_item[closest]
        isDone = rnorms_item[closest] < particle_radius
        v_hat2Item = r_closest_hat

        # if done give me the item position, else give me the vector pointing towards the item
        robsWithNearItems[i] = tuple(nearItemsList[closest]) if isDone else v_hat2Item
    return robsWithNearItems



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


#def calcTorqueFromNeigh(pos, robot_states, robRobNeig, robItemNeig, v_hat, nOfRobots, nOfItems):
#
#    torque_rob = np.zeros((nOfRobots,1))
#    torque_item = np.zeros((nOfRobots,1))
#    # calculate torque for each particle based on nbhd
#    for i in range(nOfRobots):
#
#        # if robot state is != 0 then there is no effect
#        if robot_states[i] != 0:
#            torque_rob[i] = 0
#            torque_item[i] = 0
#            continue
#
#        nOfRobotsInNeigh = len(robRobNeig[i])
#        nOfItemsInNeigh = len(robItemNeig[i])
#
#        if nOfRobotsInNeigh == 0:
#            torque_rob[i] = 0
#        else:
#            pos_neighbs = np.array([pos[n] for n in robRobNeig[i]]) if nOfRobotsInNeigh > 0 else 0
#            r_rob = pos[i] - pos_neighbs
#            rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobotsInNeigh,1))
#            r_rob_hat  = r_rob / rnorms_rob
#            dots_rob = np.sum(v_hat[i] * r_rob_hat, axis=1).reshape((nOfRobotsInNeigh,1))
#            coefs = dots_rob / rnorms_rob**2 
#            crosses = np.cross(v_hat[i], r_rob_hat).reshape((nOfRobotsInNeigh, 1))
#            particle_torques = coefs * crosses
#            torque_rob[i] = np.sum(particle_torques) 
#
#        if nOfItemsInNeigh == 0:
#            torque_item[i] = 0
#        else:
#            # calculate direction vector to things in neighbourhood
#            r_item = pos[i] - np.array(list(map(list, robItemNeig[i])))
#            # calculate the norm of every direction vector
#            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItemsInNeigh,1))
#            # collect only nearby ones
#            # we need rhat
#            r_item_hat  = r_item / rnorms_item
#            # dot 'em. dot does not support axis thing so we do it like this 
#            dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItemsInNeigh,1))
#            coefs_item = dots_item / rnorms_item**2 
#            # try repelling them now
#            # crosses v_i with r_i and does so for all i
#            crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItemsInNeigh, 1))
#            particle_torques_item = coefs_item * crosses_item
#
#            torque_item[i] = np.sum(particle_torques_item) 
#
#    return torque_rob, torque_item

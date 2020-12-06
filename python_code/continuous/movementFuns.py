import numpy as np

def getNeighbourhoods(pos, item_positions_list, nOfRobots, nOfItems, particle_radius, torque_radius):
    robRobNeig = {i:[] for i in range(nOfRobots)}
    robItemNeig = {i:[] for i in range(nOfRobots)}
    for i in range(nOfRobots):
        # calculate direction vector to every vector ( r_i,i is not a thing tho)
        r = pos[i] - pos 
        r_item = pos[i] - item_positions_list
        # calculate the norm of every direction vector
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
        # form neighbourhoods 
        robRobNeig[i] = {rob for rob in range(nOfRobots) 
                if rnorms[rob] < torque_radius and rob != i}
        robItemNeig[i] = {tuple(item_positions_list[it]) for it in range(nOfItems) if rnorms_item[it] < torque_radius}
    return robRobNeig, robItemNeig




def calcTorqueFromNeigh(pos, robot_states, robRobNeig, robItemNeig, v_hat, nOfRobots, nOfItems):

    torque_rob = np.zeros((nOfRobots,1))
    torque_item = np.zeros((nOfRobots,1))
    # calculate torque for each particle (single torque depends on all other particles) 
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




def calcTorque(pos, robot_states, item_positions_list, v_hat, nOfRobots, nOfItems, particle_radius, torque_radius):

    torque = np.zeros((nOfRobots,1))
    torque_item = np.zeros((nOfRobots,1))
    # calculate torque for each particle (single torque depends on all other particles) 
    robRobNeig = {i:[] for i in range(nOfRobots)}
    robItemNeig = {i:[] for i in range(nOfRobots)}
    for i in range(nOfRobots):
        # calculate direction vector to every vector ( r_i,i is not a thing tho)
        r = pos[i] - pos 
        r_item = pos[i] - item_positions_list
        # calculate the norm of every direction vector
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
        # collect only nearby ones
        robRobNeig[i] = [rob for rob in range(nOfRobots) if rnorms[rob] < torque_radius]
        robItemNeig[i] = {tuple(item_positions_list[it]) for it in range(nOfItems) if rnorms_item[it] < torque_radius}
        # we need rhat
        r_hat  = r / rnorms
        r_item_hat  = r_item / rnorms_item
        r_hat[i] = np.zeros(2)
        # dot 'em. dot does not support axis thing so we do it like this 
        dots = np.sum(v_hat[i] * r_hat, axis=1).reshape((nOfRobots,1))
        dots_item = np.sum(v_hat[i] * r_item_hat, axis=1).reshape((nOfItems,1))
        coefs = dots / rnorms**2 
        coefs_item = dots_item / rnorms_item**2 
        # try repelling them now
        coefs[i] = 0
        # crosses v_i with r_i and does so for all i
        crosses = np.cross(v_hat[i], r_hat).reshape((nOfRobots, 1))
        crosses_item = np.cross(v_hat[i], r_item_hat).reshape((nOfItems, 1))
        particle_torques = coefs * crosses
        particle_torques_item = coefs_item * crosses_item



        particle_torques = np.array([particle_torques[p] for p in range(nOfRobots) if rnorms[p] < torque_radius])
        particle_torques_item = np.array([particle_torques_item[p] for p in range(nOfItems) if rnorms_item[p] < torque_radius])


        torque[i] = np.sum(particle_torques) 
        torque_item[i] = np.sum(particle_torques_item) 

        if robot_states[i] == 1:
            torque[i] = 0
            torque_item[i] = 0
            continue

    return torque, torque_item, robRobNeig, robItemNeig





def calcForceAttractionRepulsion(pos, robot_states, item_positions_list, v_hat, nOfRobots, nOfItems, particle_radius, torque_radius):

    force_rob = np.zeros((nOfRobots,2))
    force_item = np.zeros((nOfRobots,2))
    # calculate torque for each particle (single torque depends on all other particles) 
    robRobNeig = {i:[] for i in range(nOfRobots)}
    robItemNeig = {i:[] for i in range(nOfRobots)}
    for i in range(nOfRobots):
        # calculate direction vector to every vector ( r_i,i is not a thing tho)
        r_rob = pos[i] - pos 
        r_item = pos[i] - item_positions_list
        # calculate the norm of every direction vector
        rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobots,1))
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
        # collect only nearby ones
        robRobNeig[i] = [rob for rob in range(nOfRobots) if rnorms_rob[rob] < torque_radius]
        robItemNeig[i] = {tuple(item_positions_list[it]) for it in range(nOfItems) if rnorms_item[it] < torque_radius}
        # we need rhat
        r_rob_hat  = r_rob / rnorms_rob
        r_item_hat  = r_item / rnorms_item
        r_rob_hat[i] = np.zeros(2)
        rnorms_rob[i] = 1

        rob_forces = np.array([r_rob_hat[p] / (rnorms_rob[p])**2 for p in range(nOfRobots) if rnorms_rob[p] < torque_radius])
        item_forces = np.array([r_item_hat[p] / (rnorms_item[p])**2 for p in range(nOfItems) if rnorms_item[p] < torque_radius])


        if len(rob_forces) > 1:
            #print(rob_forces)
            force_rob[i] = np.sum(rob_forces, axis=0) 
        else:
            force_rob[i] = np.zeros(2)

        if len(force_item) > 0:
            #print(item_forces)
            force_item[i] = np.sum(item_forces, axis=0) 
        else:
            force_item[i] = np.zeros(2)
        
        if robot_states[i] == 1:
            force_rob[i] = 0
            force_item[i] = 0
            continue

    return force_rob, force_item, robRobNeig, robItemNeig







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
def volumeExclusion(x, y, pos, step, robRobNeig, particle_radius, nOfRobots):

    for p in range(nOfRobots):
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

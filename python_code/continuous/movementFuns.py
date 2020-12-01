import numpy as np

def calcTorque(pos, robot_states, item_positions_list, v_hat, nOfRobots, nOfItems, particle_radius, torque_radius):

    torque = np.zeros((nOfRobots,1))
    torque_item = np.zeros((nOfRobots,1))
    # calculate torque for each particle (single torque depends on all other particles) 
    robRobNeig = {i:[] for i in range(nOfRobots)}
    robItemNeig = {i:[] for i in range(nOfRobots)}
    for i in range(1, nOfRobots):
        # calculate direction vector to every vector ( r_i,i is not a thing tho)
        r = pos[i] - pos 
        r_item = pos[i] - item_positions_list
        # calculate the norm of every direction vector
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
        rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
        # collect only nearby ones
        robRobNeig[i] = [rob for rob in range(nOfRobots) if rnorms[rob] < 10 * particle_radius]
        robItemNeig[i] = {tuple(item_positions_list[it]) for it in range(nOfItems) if rnorms_item[rob] < 10 * particle_radius}
        # we need rhat
        if robot_states[i] == 1:
            torque[i] = 0
            torque_item[i] = 0
            continue
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

    return torque, torque_item, robRobNeig, robItemNeig



# TODO return list of robots that are at the delivery station too
def v_hat2DeliveryStation(pos, delivery_station):
    rToDelivery = delivery_station - pos
    rnorms = np.linalg.norm(rToDelivery, axis=1)
    v_hat2DeliveryStation = rToDelivery / rnorms
    return v_hat2DeliveryStation


# makes code cleaner, but i don't want these huge x's and y's sloshing around
# test whether just references or whole objects are passed (most likely they are refs but check)
def volumeExclusion(x,y, nOfRobots, particle_radius):
        pos = np.hstack((x[:, step+1].reshape((nOfRobots,1)), 
                         y[:, step+1].reshape((nOfRobots,1))))
    for p in range(nOfRobots):
        r = pos[p] - pos 
        rnorms = np.linalg.norm(r, axis=1).reshape((nOfRobots,1))
        rnorms[p] = 'Inf'
        r_hat  = r / rnorms

        for n in robRobNeig[p]:
            if rnorms[n] < 2 * particle_radius:
                overlap = 2 * particle_radius - rnorms[n]
                moveVec = r_hat[n] * (overlap / 2)
#                    print(moveVec)
                x[p, step+1] += moveVec[0]
                y[p, step+1] += moveVec[1]
                x[n, step+1] -= moveVec[0]
                y[n, step+1] -= moveVec[1]

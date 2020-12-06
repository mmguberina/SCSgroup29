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

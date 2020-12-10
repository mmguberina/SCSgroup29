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



def separateByState(robot_states, nOfRobots):
    explorers = set() # state 0
    returners = set() # state 1
    itemPickers = set() # state 2

    for i in range(nOfRobots):
        if robot_states[i] == 0:
            explorers.add(i)
        if robot_states[i] == 1:
            returners.add(i)
        if robot_states[i] == 2:
            itemPickers.add(i)

    return explorers, returners, itemPickers




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



def getNeighbourhoodsForTesting(pos, item_positions_list, nOfRobots, nOfItems, particle_radius, torque_radius, obstacles, obstacleRadius):
    nOfObss = len(obstacles)
    robRobNeig = {i:[] for i in range(nOfRobots)}
    robItemNeig = {i:[] for i in range(nOfRobots)}
    robObsNeig = {i:[] for i in range(nOfRobots)}
    noItems = False
    noObstacles = False
    for i in range(nOfRobots):
        # calculate direction vector to every vector ( r_i,i is not a thing tho)

        if len(item_positions_list) > 0:
            r_item = pos[i] - item_positions_list
            rnorms_item = np.linalg.norm(r_item, axis=1).reshape((nOfItems,1))
            robItemNeig[i] = {tuple(item_positions_list[it]) for it in range(nOfItems) if rnorms_item[it] < torque_radius}
        else:
            noItems = True

        if len(obstacles) > 0:
            r_obs  = pos[i] - obstacles
            rnorms_obs = np.linalg.norm(r_obs, axis=1).reshape((nOfObss,1))
            robObsNeig[i] = {tuple(obstacles[o]) for o in range(nOfObss) 
                        if rnorms_obs[o] - obstacleRadius < torque_radius}
        else:
            nOfObstacles = True

        r_rob = pos[i] - pos 
        rnorms_rob = np.linalg.norm(r_rob, axis=1).reshape((nOfRobots,1))
        robRobNeig[i] = {rob for rob in range(nOfRobots) 
                if rnorms_rob[rob] < torque_radius and rob != i}

    return robRobNeig, {} if nOfItems else robItemNeig , {} if noObstacles else robObsNeig

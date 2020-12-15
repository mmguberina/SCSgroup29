import numpy as np


# NOTE the obstacles must be initialized so that they don't cover the delivery station nor it's surroundings
#   --> it can realistically be expected that an actual station won't be positioned in a bad place anyway
def initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius, delivery_station):
    coveredArea = gridSize**2 * percetangeOfCoverage
    nOfObstacles = int(coveredArea / obstacleRadius / 5)
    obstacles = np.fix(np.random.random((nOfObstacles, 2)) * gridSize)
    r_to_ds = delivery_station - obstacles
    rnorms = np.linalg.norm(r_to_ds, axis=1)
    obstacles = np.array([obstacles[i] for i in range(len(obstacles)) if rnorms[i] > 2 * obstacleRadius ])
    return obstacles


# get clusters
# store the spliced points as well (in the value ndarray)
# NOTE this is bad and it does not work
# but the idea is good so fix it eventually
def indentifyObstacleClustersWithSplits(obstacles, obstacleRadius, particle_radius):
    # store them in a dictionary
    # keep list of neighbours of every obstacle
    # you'll need to do blacklist when using this, but this will work nicely
    obstacleClusters = {tuple(obstacle):[] for obstacle in obstacles}
    nOfObstacles = len(obstacles)

    # this is in fact not completely correct in terms of indentifying clusters
    # but i think it effectively correct for the sim purposes
    # (i don't feel like making it 100% correct rn)
    for i in range(len(obstacles)):
        r_obs = obstacles[i] - obstacles
        rnorms_obs = np.linalg.norm(r_obs, axis=1)
        # i will add the obstacle itself in the value of that obstacle in the dict
        justObs = np.array([obstacles[o] for o in range(nOfObstacles) 
                                            if rnorms_obs[o] < 2*obstacleRadius + 2*particle_radius])
        # using a set as a cheap way to stop overcounting
        clusterAsSet = set()

        # and now calc and add the spliced points as well
        for j in range(len(justObs)):
            # it's clear when you draw it
            # also this thing gives you the obstacle itself, which is what we're going for here actually
            spliced = (justObs[j] - justObs) / 2 + justObs
            spliced = set(map(tuple, spliced))
            clusterAsSet.update(spliced)


        obstacleClusters[tuple(obstacles[i])] = clusterAsSet


    return obstacleClusters


def indentifyObstacleClusters(obstacles, obstacleRadius, particle_radius):
    # store them in a dictionary
    # keep list of neighbours of every obstacle
    # you'll need to do blacklist when using this, but this will work nicely
    obstacleClusters = {tuple(obstacle):[] for obstacle in obstacles}
    nOfObstacles = len(obstacles)

    # this is in fact not completely correct in terms of indentifying clusters
    # but i think it effectively correct for the sim purposes
    # (i don't feel like making it 100% correct rn)
    for i in range(len(obstacles)):
        r_obs = obstacles[i] - obstacles
        rnorms_obs = np.linalg.norm(r_obs, axis=1)
        # i will add the obstacle itself in the value of that obstacle in the dict
        justObs = np.array([obstacles[o] for o in range(nOfObstacles) 
                                            if rnorms_obs[o] < 2*obstacleRadius + 2*particle_radius])


        obstacleClusters[tuple(obstacles[i])] = set(map(tuple, justObs))


    return obstacleClusters

# NOTE items must be initialize so that they don't overlap with obstacles
# this is disgustingly bad from an efficiency standpoint
# hopefully it runs only once so let's pretend it's not possibly the worst code i've ever written
def initializeItems(nOfItems, gridSize, obstacles, obstacleRadius):
    obstaclesSet = set(map(tuple, obstacles))
    item_positions_list = np.fix(np.random.random((nOfItems, 2)) * gridSize)
    item_positions_set = set(map(tuple, item_positions_list))

    #remove em
    for i in range(nOfItems):
        r_obs = obstacles - item_positions_list[i]
        rnorms = np.linalg.norm(r_obs, axis=1)
        for j in range(len(obstacles)):
            if rnorms[j] < obstacleRadius:
                try:
                    item_positions_set.remove(tuple(item_positions_list[i]))
                except KeyError:
                    continue
    item_positions_list = np.array(list(item_positions_set))

    while len(item_positions_set) < nOfItems:
        item_positions_set.add(tuple(np.random.randint((gridSize, gridSize))))
        item_positions_list = np.array(list(item_positions_set))
        for i in range(len(item_positions_set)):
            r_obs = obstacles - item_positions_list[i]
            rnorms = np.linalg.norm(r_obs, axis=1)
            for j in range(len(obstacles)):
                if rnorms[j] < obstacleRadius:
                    try:
                        item_positions_set.remove(tuple(item_positions_list[i]))
                    except KeyError:
                        continue
        item_positions_list = np.array(list(item_positions_set))

    return item_positions_set, item_positions_list




def createCube(gridSize):
    coveredArea = gridSize**2 * percetangeOfCoverage
    nOfObstacles = int(coveredArea / obstacleRadius / 5)
    obstacles = np.fix(np.random.random((nOfObstacles, 2)) * gridSize)
    r_to_ds = delivery_station - obstacles
    rnorms = np.linalg.norm(r_to_ds, axis=1)
    obstacles = np.array([obstacles[i] for i in range(len(obstacles)) if rnorms[i] > 2 * obstacleRadius ])
    return obstacles




# TODO create a forest-like environment
def initializeForest(percetangeOfCoverage, gridSize):
    pass

import numpy as np


def initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius, delivery_station):
    coveredArea = gridSize**2 * percetangeOfCoverage
    nOfObstacles = int(coveredArea / obstacleRadius / 5)
    obstacles = np.fix(np.random.random((nOfObstacles, 2)) * gridSize)
    r_to_ds = delivery_station - obstacles
    rnorms = np.linalg.norm(r_to_ds, axis=1)
    obstacles = np.array([obstacles[i] for i in range(len(obstacles)) if rnorms[i] > 2 * obstacleRadius ])
    return obstacles
    

# this is disgustingly bad
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

def initializeForest(percetangeOfCoverage, gridSize):
    pass

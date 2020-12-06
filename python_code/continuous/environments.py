import numpy as np


def initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius):
    coveredArea = gridSize**2 * percetangeOfCoverage
    nOfObstacles = int(coveredArea / obstacleRadius / 5)
    print(nOfObstacles)

    return np.fix(np.random.random((nOfObstacles, 2)) * gridSize)
    

def initializeItems(nOfItems, gridSize):
    item_positions_list = np.fix(np.random.random((nOfItems, 2)) * gridSize)
    item_positions_set = set(map(tuple, item_positions_list))
    while len(item_positions_set) < nOfItems:
        item_positions_set.add(tuple(np.random.randint((gridSize, gridSize))))

    item_positions_list = np.array(list(item_positions_set))
    return item_positions_set, item_positions_list

def initializeForest(percetangeOfCoverage, gridSize):
    pass

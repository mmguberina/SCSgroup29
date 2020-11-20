import numpy as np
from collections import deque
import copy

# should the robot keep a list of all items in it's visibility sphere?
# if the visibility sphere is larger than the amount it can move,
# then we don't need to reisntantiate the list at every time step


# when a robot picks up an item,
# we would use A* to find the shortest path to the delivery bin

class Robot:
    """ 
    this class exists just for compartmentalization purposes.
    everything should be self-explanatory
    """
    def __init__(self, position, state, gridSize, visibilityRadius):
        self.position = position
        self.state = state
        self.gridSize = gridSize
        self.storage = []
        self.visibilityRadius = visibilityRadius


    def __str__(self):
        if self.individualState == 0:
            state = 'Searching'
        if self.individualState == 1:
            state = 'Delivering'
        if self.individualState == 2:
            state = 'Ready to empty storage'
        return str(self.position) + ',' + state + '\n'

    def __repr__(self):
        return str(self)

    def __hash__(self):
        return hash(tuple(self.position))

    def moveUp(self):
        if self.position[1] < self.gridSize[1]:
            self.position[1] += 1

    def moveDown(self):
        if self.position[1] > 0:
            self.position[1] -= 1


    def moveRight(self):
        if self.position[0] < self.gridSize[0]:
            self.position[0] += 1

    def moveLeft(self):
        if self.position[0] > 0:
            self.position[0] -= 1

    def moveRandomly(self):
        outcome = np.random.randint(4)
        if outcome == 0:
            self.moveUp()
        if outcome == 1:
            self.moveDown()
        if outcome == 2:
            self.moveRight()
        if outcome == 3:
            self.moveLeft()
    
    def getPosition(self):
        return tuple(self.position)

    def getVisibilitySphere(self):
        visibilitySphere = {}
        toVisit = deque()
        deque.append(self.position)
        visibilitySphere.add(tuple(self.position))
        # the range is the number of total points of distance < visibilityRadius
        # the formula is easy to derive
        for i in range(2 * self.visibilityRadius * (self.visibilityRadius + 1)):
            currentNode = toVisit.popleft()
            neighbors = []
            neighbors.append(currentNode + [0,1])
            neighbors.append(currentNode + [1,0])
            neighbors.append(currentNode - [0,1])
            neighbors.append(currentNode - [1,0])
            for n in neighbors:
                if n not in visibilitySphere:
                    toVisit.append(n)
                    visibilitySphere.add(tuple(n))
        return visibilitySphere


    def emptyStorage(self):
        self.state = 1
        items = copy.deepcopy(self.storage)
        self.storage = []
        return items


    def moveToSpecificPoint(self, point):
        direction = point - self.position
        if direction[0] == 0 and direction[1] == 0:
            self.state == 2
            return
        outcome = np.random.random()
        if outcome < 0.5:
            if direction[0] != 0:
                self.position[0] += np.sign(direction[0])
            else:
                self.position[1] += np.sign(direction[1])
        else:
            if direction[1] != 0:
                self.position[1] += np.sign(direction[1])
            else:
                self.position[0] += np.sign(direction[0])

    def movePSOStyle(self, surroundingObjects):
        pass



class Item:
    """ 
    this class exists just for compartmentalization purposes.
    everything should be self-explanatory
    """
    def __init__(self, position):
        self.position = position
    def position(self):
        return tuple(self.position)
    def __hash__(self):
        return __hash__(tuple(self.position))

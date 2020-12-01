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
    def __init__(self, position, state, gridSize, 
            visibilityRadius=None, repulsionF=None, attractionF=None):
        self.position = position
        self.state = state
        self.gridSize = gridSize
        self.storage = []
        if visibilityRadius != None:
            self.visibilityRadius = visibilityRadius
        if repulsionF != None:
            self.repulsionF = repulsionF
        if attractionF!= None:
            self.attractionF = attractionF


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

    def move(self, probs=None):
        if probs == None:
            outcome = np.random.randint(4)
            if outcome == 0:
                self.moveUp()
            if outcome == 1:
                self.moveDown()
            if outcome == 2:
                self.moveRight()
            if outcome == 3:
                self.moveLeft()

        else:
            outcome = np.random.random()
            if outcome < probs[0]:
                self.moveUp()
            if outcome >= probs[0] and outcome < probs[1]:
                self.moveDown()
            if outcome >= probs[1] and outcome < probs[2]:
                self.moveRight()
            if outcome >= probs[2]:
                self.moveLeft()
    
    def getPosition(self):
        return tuple(self.position)

    def getVisibilitySphere(self):
        visibilitySphere = set()
        toVisit = deque()
        toVisit.append(self.position)
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
                if tuple(n) not in visibilitySphere:
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
            self.state = 2
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


    
    def movePSOStyle(self, nearRobots, nearItems):
        """
        the here is to add forces and then normalize everything to
        probabilities. no idea whether this is the right way to do it tho.
        maybe it should be more like in the active swimmers case from csc hw3.
        that certainly seems like the correct approach for the continuous case
        """
        upProb, downProb, rightProb, leftProb = [0.25] * 4;
        for item in nearItems:
            vecToItem = self.position
            vecToItem[0] -= item[1]
            vecToItem[1] -= item[1]
            distanceToItem = np.abs(vecToItem[0]) + np.abs(vecToItem[1])

            if vecToItem[0] >= 0 and vecToItem[1] >= 0:
                rightProb += self.attractionF * vecToItem[0]/distanceToItem
                upProb += self.attractionF *vecToItem[1]/distanceToItem

            if vecToItem[0] <= 0 and vecToItem[1] >= 0:
                leftProb += self.attractionF *np.abs(vecToItem[0])/distanceToItem
                upProb += self.attractionF *vecToItem[1]/distanceToItem

            if vecToItem[0] <= 0 and vecToItem[1] <= 0:
                leftProb += self.attractionF *np.abs(vecToItem[0])/distanceToItem
                downProb += self.attractionF *np.abs(vecToItem[1])/distanceToItem

            if vecToItem[0] >= 0 and vecToItem[1] <= 0:
                rightProb += self.attractionF *vecToItem[0]/distanceToItem
                downProb += self.attractionF *np.abs(vecToItem[1])/distanceToItem

        # in order not to have negative values, just add to the opposite direction
        # hopefully that makes sense
        for robot in nearRobots:
            vecToRobot = copy.deepcopy(self.position)
            vecToRobot[0] -= robot[1]
            vecToRobot[1] -= robot[1]
            distanceToRobot = np.abs(vecToRobot[0]) + np.abs(vecToRobot[1])

            if vecToRobot[0] >= 0 and vecToRobot[1] >= 0:
          #      rightProb -= self.repulsionF * vecToRobot[0]/distanceToRobot
          #      upProb -= self.repulsionF * vecToRobot[1]/distanceToRobot
                leftProb += self.repulsionF * vecToRobot[0]/distanceToRobot
                downProb += self.repulsionF * vecToRobot[1]/distanceToRobot


            if vecToRobot[0] <= 0 and vecToRobot[1] >= 0:
                #leftProb -= self.repulsionF * np.abs(vecToRobot[0])/distanceToRobot
                #upProb -= self.repulsionF * vecToRobot[1]/distanceToRobot
                rightProb += self.repulsionF * np.abs(vecToRobot[0])/distanceToRobot
                downProb += self.repulsionF * vecToRobot[1]/distanceToRobot

            if vecToRobot[0] <= 0 and vecToRobot[1] <= 0:
                #leftProb -= self.repulsionF * np.abs(vecToRobot[0])/distanceToRobot
                #downProb -= self.repulsionF * np.abs(vecToRobot[1])/distanceToRobot
                rightProb += self.repulsionF * np.abs(vecToRobot[0])/distanceToRobot
                upProb += self.repulsionF * np.abs(vecToRobot[1])/distanceToRobot

            if vecToRobot[0] >= 0 and vecToRobot[1] <= 0:
                #rightProb -= self.repulsionF * vecToRobot[0]/distanceToRobot
                #downProb -= self.repulsionF * np.abs(vecToRobot[1])/distanceToRobot
                leftProb += self.repulsionF * vecToRobot[0]/distanceToRobot
                upProb  += self.repulsionF * np.abs(vecToRobot[1])/distanceToRobot

        # and now we need to normalize to have probabilities
        sumOfProbs = upProb + downProb + rightProb + leftProb
        probs = np.array([upProb, downProb, rightProb, leftProb]) / sumOfProbs
        self.move(probs=list(probs))
        



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

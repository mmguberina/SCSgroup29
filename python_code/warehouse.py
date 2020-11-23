import numpy as np
from robot import *
"""
this class implements the model.
it is used to store global information which is used for plots
and from it robots can sense their surrounding
even though every piece of information is stored here,
the robots only have access to their surroundings
"""
class Warehouse:
    def __init__(self, populationSize, gridSize, packageAppearanceProb, delivery_station,
            visibilityRadius=None, attractionF=None, repulsionF=None):
        self.populationSize = populationSize
        self.gridSize = gridSize
        self.delivery_station = np.array(delivery_station)


        """
        the positions of robots and items are stored as hashmaps* because:
        a) there is no need to keep empty squares in memory
        b) their contents will have to be constantly checked (is something on square (x,y))
        and hash maps do that in O(1) time, while python lists are dynamic lists
        and thus it takes O(n) time to check whether ith item is in there

        * note:
        afaik, this is how sparse matrices are implemented as well (0s are not stored),
        it's just that packages which provide sparse matrices also implement
        the usual matrix operations (which we do not need). in other words, if
        you don't know about hashmaps (dictionaries in python), just think that
        the 2D grid (Warehouse floor) is stored as a sparse matrix.

        all information is stored by the robot and item classes (but function just
        as structs would, the only point with having this is to have 
        all relevant info of a robot/item accessible in the same place).

        as the robots will move at each iteration, we will need to 
        update all positions at every iterations, so a hashmap of their 
        positions will have to be created at every timestep.
        the items won't move unless picked up so we can store their positions
        and only change that hashmap as needed.
        """

        # initialize robots 

        # generate unique positions
        positions = set(map(tuple, np.fix(np.random.random((self.populationSize,2)) * self.gridSize)))
        # keep adding position until we have the desired n of robots
        while len(positions) < self.populationSize:
            positions.add((np.random.randint(self.gridSize[0]), np.random.randint(self.gridSize[1])))
        positions = np.array(list(map(list, positions)))
        individualStates = np.zeros(self.populationSize)
        # store robots in a list
        if visibilityRadius == None:
            self.robots = [Robot(positions[i], individualStates[i], gridSize, 3) 
                    for i in range(populationSize)]
        else:
            self.robots = [Robot(positions[i], individualStates[i], gridSize, 
                visibilityRadius=visibilityRadius, 
                attractionF=attractionF, repulsionF=repulsionF) 
                    for i in range(populationSize)]

        self.robot_positions = self.getRobotPositions()

        # initialize items
        item_positions = set(map(tuple, np.fix(np.random.random((self.populationSize,2)) * self.gridSize)))
        while len(item_positions) < self.populationSize:
            item_positions.add((np.random.randint(self.gridSize[0]), np.random.randint(self.gridSize[1])))
        # the items won't move so we can keep them in a set
        self.item_positions = item_positions
        self.items = [Item(p) for p in item_positions]


    def getRobotPositions(self):
        # not we return a set
        return {robot.getPosition() for robot in self.robots}


    # write the controler in another file, not here!
    # make stuff separated
    # you don't actually want to open this file unless changing the model
#    def randomSearchRobotControl(self):
#        robot_positions = []



    def generateItems(self):
        outcomes = np.random.random((self.gridSize[0], self.gridSize[1]))
        outcomes = outcomes - self.growthProb
        for i in range(self.gridSize[0]):
            for j in range(self.gridSize[1]):
                if outcome[i][j] < self.growthProb:
                    self.item_positions.add((i,j))

    def checkVisibilitySphere(self, visibilitySphere):
        robots = {i for i in visibilitySphere if i in self.robot_positions}
        items = {i for i in visibilitySphere if i in self.item_positions}
        return robots, items
        




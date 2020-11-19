import numpy as np


class Robot:
    def __init__(self, position, individualState, gridSize):
        self.position = position
        self.individualState = individualState
        self.gridSize = gridSize

    def __str__(self):
        if self.individualState == 0:
            state = 'Searching'
        if self.individualState == 1:
            state = 'Delivering'
        return str(self.position) + ',' + state + '\n'

    def __repr__(self):
        return str(self)

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

    def moveRandomly(self, gridSize):
        outcome = np.random.randint(4)
        if outcome == 0:
            self.moveUp()
        if outcome == 1:
            self.moveDown()
        if outcome == 2:
            self.moveRight()
        if outcome == 3:
            self.moveLeft()


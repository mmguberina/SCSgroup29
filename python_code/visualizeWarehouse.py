from robot import *
from warehouse import *
import numpy as np
import matplotlib.pyplot as plt

def plotWarehouseSnapshot(ax, warehouse, nOfDeliveredItems=None, camera=None):

    robotArray = np.array(list(warehouse.getRobotPositions()))
    itemArray = np.array(list(warehouse.item_positions))

    ax.scatter(robotArray[:,0], robotArray[:,1], c='tab:red', marker="D")
    ax.scatter(itemArray[:,0], itemArray[:,1], c='tab:blue', marker="x")
    ax.scatter(warehouse.delivery_station[0], warehouse.delivery_station[1], c='tab:green', marker="o")

    if nOfDeliveredItems != None:
        ax.set_title("Thus far " + str(nOfDeliveredItems) + "were delivered")

    if camera == None:
        plt.show()
    else:
        camera.snap()


#def animateForest(forest, ax, camera):
#
#    for iteration in range(300):
#        forest.growTrees()
#        lit = forest.lightningStrike()
#
#        treeArray = np.array(list(map(list, forest.positions)))
#        litArray = np.array(list(map(list, lit)))
#
#
#        if iteration > 10:
#            ax.scatter(treeArray[:,0], treeArray[:,1], c='tab:green', marker="x")
#            ax.scatter(litArray[:,0], litArray[:,1], c='tab:red', marker="D")
#            camera.snap()
#

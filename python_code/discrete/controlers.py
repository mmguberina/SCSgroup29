import numpy as np
from robot import *
from warehouse import *
from visualizeWarehouse import *
from celluloid import Camera

def randomMovers(warehouse, ax, camera):
    # this needs to be redone so that all robots move at once
    for i in range(100):
        nOfDeliveredItems = 0
        robot_positions = warehouse.getRobotPositions()

        for robot in warehouse.robots:
            if robot.getPosition() in warehouse.item_positions:
                robot.storage.append(robot.getPosition())
                warehouse.item_positions.remove(robot.getPosition())
                robot.state = 1

            if robot.state == 2:
                nOfDeliveredItems += len(robot.emptyStorage())
                robot.state = 0

            if robot.state == 0:
                robot.move()
            else:
                robot.moveToSpecificPoint(warehouse.delivery_station)

        plotWarehouseSnapshot(ax, warehouse, nOfDeliveredItems, camera)


def PSOlikeMovers(warehouse, ax, camera):
    # this needs to be redone so that all robots move at once
    for i in range(100):
        nOfDeliveredItems = 0
        robot_positions = warehouse.getRobotPositions()

        for robot in warehouse.robots:
            if robot.getPosition() in warehouse.item_positions:
                robot.storage.append(robot.getPosition())
                warehouse.item_positions.remove(robot.getPosition())
                robot.state = 1

            if robot.state == 2:
                nOfDeliveredItems += len(robot.emptyStorage())
                robot.state = 0

            if robot.state == 0:
                visibilitySphere = robot.getVisibilitySphere()
                nearRobots, nearItems = warehouse.checkVisibilitySphere(visibilitySphere)
                robot.movePSOStyle(nearRobots, nearItems)
            else:
                robot.moveToSpecificPoint(warehouse.delivery_station)

        plotWarehouseSnapshot(ax, warehouse, nOfDeliveredItems, camera)

if __name__ == "__main__":

    fig, ax = plt.subplots()
    camera = Camera(fig)

    visibilityRadius = 10
    attractionF = 2
    repulsionF = 2

    warehouse = Warehouse(10, [20,30], 0.1, [10,10], visibilityRadius=visibilityRadius,
            attractionF=attractionF, repulsionF=repulsionF)
#    PSOlikeMovers(warehouse, ax, camera)
    randomMovers(warehouse, ax, camera)

    animation = camera.animate()
    animation.save("somerun.mp4")
    plt.show()



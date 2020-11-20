import numpy as np
from robot import *
from warehouse import *
from visualizeWarehouse import *
from celluloid import Camera

#def randomWalkers(warehouse):

fig, ax = plt.subplots()
camera = Camera(fig)

warehouse = Warehouse(5, [20,30], 0.1, [10,10])


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

        if robot.state == 0:
            robot.moveRandomly()
        else:
            robot.moveToSpecificPoint(warehouse.delivery_station)

    plotWarehouseSnapshot(ax, warehouse, nOfDeliveredItems, camera)

animation = camera.animate()
animation.save("somerun.mp4")
plt.show()






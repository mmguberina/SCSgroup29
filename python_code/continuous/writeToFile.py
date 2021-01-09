

def writeToFilee(dataFileName, N, nOfRobots, gridSize, v, particle_radius, torque_radius,\
        obstacleRadius, walkType, ni, deviation, T0, FR0, FI0, FO0,TR0, TO0,\
        nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance, percetangeOfCoverage, \
        delivery_station, obstacles, item_positions_set,\
        nOfCollectedItemsPerTime, item_positions_listPerTime):


    dataFile = open(dataFileName + '_rest.csv', 'w')
    dataFile.write("dataFileName " + str(dataFileName) + "\n")
    dataFile.write("N " + str(N) + "\n")
    dataFile.write("nOfRobots " + str(nOfRobots) + "\n")
    dataFile.write("gridSize " + str(gridSize) + "\n")
    dataFile.write("v " + str(v) + "\n")
    dataFile.write("particle_radius " + str(particle_radius) + "\n")
    dataFile.write("torque_radius " + str(torque_radius) + "\n")
    dataFile.write("obstacleRadius " + str(obstacleRadius) + "\n")
    dataFile.write("walkType " + str(walkType) + "\n")
    dataFile.write("ni " + str(ni) + "\n")
    dataFile.write("deviation " + str(deviation) + "\n")
    dataFile.write("T0 " + str(T0) + "\n")
    dataFile.write("FR0 " + str(FR0) + "\n")
    dataFile.write("FI0 " + str(FI0) + "\n")
    dataFile.write("FO0 " + str(FO0) + "\n")
    dataFile.write("TR0 " + str(TR0) + "\n")
    dataFile.write("TO0 " + str(TO0) + "\n")
    dataFile.write("nOfUnstuckingSteps " + str(nOfUnstuckingSteps) + "\n")
    dataFile.write("stuckThresholdTime " + str(stuckThresholdTime) + "\n")
    dataFile.write("stuckThresholdDistance " + str(stuckThresholdDistance) + "\n")
    dataFile.write("percetangeOfCoverage " + str(percetangeOfCoverage) + "\n")
    dataFile.write("delivery_station " + str(delivery_station) + "\n")
#    dataFile.write("obstacles " + str(obstacles) + "\n")
#    dataFile.write("item_positions_set " + str(item_positions_set) + "\n")
    dataFile.write("nOfCollectedItemsPerTime: " + "\n")
    for timestamp, nOfIt in np.array(nOfCollectedItemsPerTime):
        dataFile.write(str(timestamp) + "," + str(nOfIt) + "\n")
#    dataFile.write("item_positions_listPerTime " + str(item_positions_listPerTime) + "\n")

    dataFile.close()



def readFromFile(dataFileName):
    dataFile = open(dataFileName + '_rest.csv', 'r')
    data = {}

    line = 'thing'

    while line != '':
        line = dataFile.readline().strip().split(" ")
        print(line)
        data[line[0]] = line[1]

    return data


#    dataFile.write("dataFileName " + str(dataFileName) + "\n")
#    dataFile.write("N " + str(N) + "\n")
#    dataFile.write("nOfRobots " + str(nOfRobots) + "\n")
#    dataFile.write("gridSize " + str(gridSize) + "\n")
#    dataFile.write("v " + str(v) + "\n")
#    dataFile.write("particle_radius " + str(particle_radius) + "\n")
#    dataFile.write("torque_radius " + str(torque_radius) + "\n")
#    dataFile.write("obstacleRadius " + str(obstacleRadius) + "\n")
#    dataFile.write("walkType " + str(walkType) + "\n")
#    dataFile.write("ni " + str(ni) + "\n")
#    dataFile.write("deviation " + str(deviation) + "\n")
#    dataFile.write("T0 " + str(T0) + "\n")
#    dataFile.write("FR0 " + str(FR0) + "\n")
#    dataFile.write("FI0 " + str(FI0) + "\n")
#    dataFile.write("FO0 " + str(FO0) + "\n")
#    dataFile.write("TR0 " + str(TR0) + "\n")
#    dataFile.write("TO0 " + str(TO0) + "\n")
#    dataFile.write("nOfUnstuckingSteps " + str(nOfUnstuckingSteps) + "\n")
#    dataFile.write("stuckThresholdTime " + str(stuckThresholdTime) + "\n")
#    dataFile.write("stuckThresholdDistance " + str(stuckThresholdDistance) + "\n")
#    dataFile.write("percetangeOfCoverage " + str(percetangeOfCoverage) + "\n")
#    dataFile.write("delivery_station " + str(delivery_station) + "\n")
#    dataFile.write("obstacles " + str(obstacles) + "\n")
#    dataFile.write("item_positions_set " + str(item_positions_set) + "\n")
#    dataFile.write("nOfCollectedItemsPerTime " + str(nOfCollectedItemsPerTime) + "\n")
#    dataFile.write("item_positions_listPerTime " + str(item_positions_listPerTime) + "\n")

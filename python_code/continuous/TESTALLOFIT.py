import numpy as np
from scipy.stats import norm
from environments import *
from runSim import *
from writeToFile import *
import pandas as pd
import multiprocessing as mp
from queue import Empty
import os


def do5Tests(jobQueue):
    while True:
        try:
            params = jobQueue.get(False)
        except Empty:
            return

        print("Process", os.getpid(), "running", params)

        N, nOfRobots, gridSize, v, particle_radius, torque_radius,\
        obstacleRadius, walkType, ni, deviation, T0, FR0, FI0, FO0,TR0, TO0,\
        nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance, percetangeOfCoverage = params

        # run 5 tests to get at least some averaging
        for i in range(5):
            print("doing the", i+1, "test for given params")

            x = np.zeros((1 * nOfRobots,N+1))
            x[:,0] = 2 * np.random.random(nOfRobots) - 1 + gridSize // 2
            y = np.zeros((1 * nOfRobots,N+1))
            y[:,0] = 2 * np.random.random(nOfRobots) - 1 + gridSize // 2
            robot_statesPerTime = np.zeros((1 * nOfRobots,N+1))
            delivery_station = np.array([gridSize // 2, gridSize // 2])
            obstacles = initializeRandom(percetangeOfCoverage, gridSize, obstacleRadius, delivery_station)
            item_positions_set, item_positions_list = initializeItems(nOfItems, gridSize, obstacles, obstacleRadius)

            x, y, nOfCollectedItemsPerTime, item_positions_listPerTime = \
                runSim(x, y, item_positions_set, delivery_station, N, nOfRobots, gridSize,  
                        robot_statesPerTime, v, particle_radius, 
                        torque_radius, obstacles,obstacleRadius, 
                        walkType, ni, deviation, 
                        T0, FR0, FI0, FO0,TR0, TO0,  
                        nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance)


            dataframe_x = pd.DataFrame({'x':x, 'y':y, 'robstate':robot_statesPerTime})
            dataframe_y = pd.DataFrame({'x':x, 'y':y, 'robstate':robot_statesPerTime})
            dataframe_rs = pd.DataFrame({'x':x, 'y':y, 'robstate':robot_statesPerTime})

            dataFileName = "./data/" + str(nOfRobots) + "_" + str(walkType) + str(ni) + "_" \
                            + str(deviation) + "_" + str(percetangeOfCoverage) \
                            + "_" + str(i+1)
                           
            dataframe_x.to_csv(dataFileName + "_x.csv")
            dataframe_y.to_csv(dataFileName + "_y.csv")
            dataframe_rs.to_csv(dataFileName + "_st.csv")

            writeToFilee(dataFileName, N, nOfRobots, gridSize, v, particle_radius, torque_radius,\
        obstacleRadius, walkType, ni, deviation, T0, FR0, FI0, FO0,TR0, TO0,\
        nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance, percetangeOfCoverage, \
        delivery_station, obstacles, item_positions_set,\
        nOfCollectedItemsPerTime, item_positions_listPerTime)

        # now write everything in files
    # write everything time-dapendent into the data frame,
    #   and write all else into a separate plain-text file
    # it will work if you put it all in a dictionary!







if __name__ == '__main__':

    nsOfRobots = [i for i in range(5,80,15)]
    #rot_dif_T = 0.2
    #trans_dif_T = 0.2
    #v = 1
    nis = np.pi * np.array([1/i for i in range(5,500,100)])
    #v = 0.05
    v = 0.3
    # Total time.

    # TODO PLAY WITH THESE VALUES SEE WHAT HAPPENS TODO
    obstacleRadius = 30
    #gridSizes = [500, 1000, 1500]
    gridSizes = [1200]
    percetangesOfCoverage = [0.04, 0.01, 0.001]
    T0 = 1
    particle_radius = 5
    torque_radius = 100 


    FI0 = 0.3#0.5
    FR0 = 1.0
    FO0 = 0.5

    TI0 = 0.0#0.5
    TR0 = 0.5
    TO0 = 0.0


    #FO0 = 1.0
    deviations = [1/i for i in range(1, 100,20)]
    deviation =0
    ni=0

    nOfUnstuckingSteps = 700
    stuckThresholdTime = 300
    stuckThresholdDistance = v * 300

    N = 14000
    #obstacleClusters = indentifyObstacleClusters(obstacles, obstacleRadius, particle_radius)


    walkTypes = ['levyFlight', 'activeSwimming','brownianMotion']

    #nsOfItems = [i for i in range(10,90,20)]
    nsOfItems = [40]
    jobs = []

    for gridSize in gridSizes:
        for nOfRobots in nsOfRobots:
            for percetangeOfCoverage in percetangesOfCoverage:
                for nOfItems in nsOfItems:
                    for walkType in walkTypes:
                        if walkType == 'levyFlight':
                            #jobs.append(1)
                            jobs.append((N, nOfRobots, gridSize,v, 
                                particle_radius, torque_radius, obstacleRadius, 
                                walkType, ni, deviation, T0, FR0, FI0, FO0,TR0, TO0,  
                                nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance,
                                percetangeOfCoverage))
                        if walkType == 'activeSwimming':
                            for ni in nis:
                                jobs.append((N, nOfRobots, gridSize,v, 
                                    particle_radius, torque_radius, obstacleRadius, 
                                    walkType, ni, deviation, T0, FR0, FI0, FO0,TR0, TO0,  
                                    nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance,
                                    percetangeOfCoverage))
                        if walkType == 'brownianMotion':
                            for deviation in deviations:
                                jobs.append((N, nOfRobots, gridSize,v, 
                                    particle_radius, torque_radius, obstacleRadius, 
                                    walkType, ni, deviation, T0, FR0, FI0, FO0,TR0, TO0,  
                                    nOfUnstuckingSteps, stuckThresholdTime, stuckThresholdDistance,
                                    percetangeOfCoverage))
    print(len(jobs))


# desktops take 50
# laptop takes 50
# laptop takes the rest,i.e. 65 

    nOfProcesses = 4
    jobQueue = mp.Queue()
    jobs = jobs[0:50]
    for job in jobs:
        jobQueue.put(job)

    listOfProcesses = []
    for _ in range(nOfProcesses):
        p = mp.Process(target=do5Tests, args=(jobQueue,))
        p.start()
        listOfProcesses.append(p)

    for _ in range(nOfProcesses):
        p.join()

#    while True:
#        try:
#            resultList = resultQueue.get(False)
#        except Empty:
#            break



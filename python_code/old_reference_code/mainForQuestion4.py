from model import *
#from celluloid import Camera
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
#matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import multiprocessing as mp
import os
from queue import Empty
import numpy as np


def getBetaGammaRelationship(infectionRateQueue, resultQueue):
    while True:
        try:
            infectionRate = infectionRateQueue.get(False)
        except Empty:
            resultQueue.put(resultList)
            dataFile.close()
            return
        populationSize= 1000
        # make it a square grid
        gridSize = 100
        diffusionRate = 0.8
        nOfStartingCases = 10
        gammas = []
        recoveredPerGamma = []
        step = 0.001
        recoveryRate = 0.001
        nOfEstimates = 10
        resultList = [infectionRate]
        dataFile = open('./data/recoveredPerGammaWithBeta' + str(infectionRate) + '.csv', 'w')
        while recoveryRate < 0.01:
            print("Process", os.getpid(), "running estimate on recovery rate:", recoveryRate,
                    "with infection rate", infectionRate)
            recoveredAvg = 0
            for i in range(nOfEstimates):
                population = Population(diffusionRate, infectionRate, recoveryRate, populationSize, gridSize, nOfStartingCases)
                population.runDynamicsTillEnd()
                infected, susceptible, recovered = population.getPopulationSIR()
                # if simulation is over 'cos no infeted, this is true
                # if simulation is over because noone is susceptible, then it is also true
                # and the simulation is over in either of these cases
                recoveredAvg += recovered + infected
#                print("did", i+1, "estimates, got", recoveredAvg /(i+1), "avg recovered")

            recoveredAvg = recoveredAvg / nOfEstimates
            dataFile.write(str(infectionRate) + "," + str(recoveredAvg) + "," + str(recoveryRate) + "\n")
            resultList.append([recoveredAvg, recoveryRate])
            gammas.append(recoveryRate)
            recoveredPerGamma.append(recoveredAvg)
            recoveryRate = recoveryRate + step




infectionRateStart = 0.025
infectionRateStep = 0.025


if __name__ == '__main__':
    nOfProcesses = 6
    infectionRateQueue = mp.Queue()
    resultQueue = mp.Queue()

    for i in range(0, int(1 / infectionRateStep)):
        infectionRate = infectionRateStart + i * infectionRateStep
        infectionRateQueue.put(infectionRate)

    listOfProcesses = []
    for _ in range(nOfProcesses):
        p = mp.Process(target=getBetaGammaRelationship, args=(infectionRateQueue, resultQueue,))
        p.start()
        listOfProcesses.append(p)

    for _ in range(nOfProcesses):
        p.join()


    fig = plt.figure()
    ax = fig.gca(projection='3d')
    while True:
        try:
            resultList = resultQueue.get(False)
        except Empty:
            break
        infectionRates = []
        recoveryRates = []
        susceptibles = []
        infectionRate = resultList.pop()
        for i in range(len(resultList)):
            infectionRates.append(infectionRate)
            recoveryRates.append(resultList[i][1])
            susceptibles.append(resultList[i][0])

        infectionRates = np.array(infectionRates)
        recoveryRates = np.array(recoveryRates)
        recoveryRates = infectionRate / recoveryRates
        susceptibles = np.array(susceptibles)
        ax.plot(infectionRates, recoveryRates, susceptibles)

    plt.savefig('beta-gama_surfaceSmallerGammas.png')
    plt.show()







## now load data for the other graph
#fil = open('./data/recoveredPerGamma.csv', 'r')
#lines = fil.readlines()
#
#gammas = []
#recoveredAvgs = []
#
#for line in lines:
#    line = line.strip().split(",")
#    recoveredAvgs.append(float(line[0]))
#    gammas.append(float(line[1]))
#fil.close()
#
#fig, ax = plt.subplots()
#
#ax.set_xlabel('recovery rate')
#ax.set_ylabel('final number of recovered agents')
#ax.set_title(r'd = ' + str(diffusionRate) + r', $\beta =$ ' + str(infectionRate) 
#        + ', starting number of infected = 1\%')
#plt.plot(gammas, recoveredAvgs, c="tab:red")
#plt.savefig('nOfRecoveredPerGamma.png')
#plt.show()
#

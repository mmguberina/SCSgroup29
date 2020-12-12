from model import *
from visualizeForest import *
from celluloid import Camera
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import pandas as pd
import multiprocessing as mp
import os
from queue import Empty



def question2(jobQueue):

    while True:
        try:
            params = jobQueue.get(False)
            gridSize, lightningStrikeProb, growthProb, torus = params
        except Empty:
            return

        regularRelativeFireSizes = []
        oneTryRelativeFireSizes = []
        nOfRunsTotal = 10000
        nOfRuns = 0
        forest = Forest(growthProb, lightningStrikeProb, 
                gridSize, torus=torus)
        print("Process", os.getpid(), "running", params)
        for i in range(nOfRunsTotal):
            forest.growTrees()
            lit = forest.lightningStrike()
            fireSize = len(lit)
            if fireSize > 0:
                nOfRuns += 1 
                nOfTreesBeforeFire = len(forest.positions) + fireSize
                regularRelativeFireSizes.append(fireSize / nOfTreesBeforeFire)

                # now repeat for a new random forest with the same number of trees
                # make sure that you get a fire
                newForest = Forest(growthProb, 1, gridSize, nOfTreesBeforeFire, torus=torus)
                newLit = newForest.lightningStrike(ensurance=1)
                newFireSize = len(newLit)
                oneTryRelativeFireSizes.append(newFireSize / nOfTreesBeforeFire)
            else:
                continue

        regularRelativeFireSizes = sorted(regularRelativeFireSizes, reverse=True)
        oneTryRelativeFireSizes = sorted(oneTryRelativeFireSizes, reverse=True)

        regularFireData = np.array([[(i+1) / nOfRuns, regularRelativeFireSizes[i]] 
                                     for i in range(nOfRuns)]) 
        oneTryFireData = np.array([[(i+1) / nOfRuns, oneTryRelativeFireSizes[i]] 
                                     for i in range(nOfRuns)]) 

        dataframe1 = pd.DataFrame(regularFireData)
        dataframe2 = pd.DataFrame(oneTryFireData)

        dataFileReg = "data/regFireDatGrid_" + str(gridSize) \
                         + "_lightProb_" +  str(lightningStrikeProb) \
                         + "_growth_" +  str(growthProb) \
                         + "_torus" +  str(torus) + ".csv"

        dataFileOneTry = "data/onetryFireDatGrid_" + str(gridSize) \
                         + "_lightProb_" +  str(lightningStrikeProb) \
                         + "_growth_" +  str(growthProb) \
                         + "_torus" +  str(torus) + ".csv"

        dataframe1.to_csv(dataFileReg)
        dataframe2.to_csv(dataFileOneTry)

#    fig, ax = plt.subplots()
#    ax.set_xscale('log')
#    ax.set_yscale('log')
#    ax.scatter(regularFireData[:,0], regularFireData[:,1], c='tab:green')
    #ax.set_xscale('log')
    #ax.set_yscale('log')
#    ax.scatter(oneTryFireData[:,0], oneTryFireData[:,1], c='tab:red')

#    plt.savefig("2rank_freq_plotsTorus.png")
#    plt.show()

if __name__ == '__main__':
    gridSizes = [8, 16, 32, 64, 128, 256, 512]
    lightningStrikeProbs = [0.1, 0.5, 0.8]
    growthProbs = [0.005, 0.02, 0.1]

    jobs = []
    for gridSize in gridSizes:
        for lightningStrikeProb in lightningStrikeProbs:
            for growthProb in growthProbs:
                for i in range(2):
                    if i == 0:
                        jobs.append((gridSize, lightningStrikeProb, growthProb, None))
                    else:
                        jobs.append((gridSize, lightningStrikeProb, growthProb, 1))
    #forest = Forest(growthProb, lightningStrikeProb, gridSize, torus=1)

    nOfProcesses = 4
    jobQueue = mp.Queue()
    for job in jobs:
        jobQueue.put(job)

    listOfProcesses = []
    for _ in range(nOfProcesses):
        p = mp.Process(target=question2, args=(jobQueue,))
        p.start()
        listOfProcesses.append(p)

    for _ in range(nOfProcesses):
        p.join()

    while True:
        try:
            resultList = resultQueue.get(False)
        except Empty:
            break



#findBigFire(forest)

#fig, ax = plt.subplots()
#camera = Camera(fig)
#animateForest(forest, ax, camera)
#animation = camera.animate()
#animation.save('somerun.mp4')
#plt.show()



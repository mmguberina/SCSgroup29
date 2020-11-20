from model import *
from celluloid import Camera
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt


def getBetaGammaRelationship(infectionRate):
    populationSize= 1000
    # make it a square grid
    gridSize = 100
    diffusionRate = 0.8
    nOfStartingCases = 10
    gammas = []
    recoveredPerGamma = []
    step = 0.002
    recoveryRate = 0.001
    nOfEstimates = 10
    dataFile = open('./data/recoveredPerGammaWithBeta' + str(infectionRate) + '.csv', 'w')
    while recoveryRate < 0.1:
        print("running estimate on recovery rate:", recoveryRate)
        recoveredAvg = 0
        for i in range(nOfEstimates):
            population = Population(diffusionRate, infectionRate, recoveryRate, populationSize, gridSize, nOfStartingCases)
            population.runDynamicsTillEnd()
            infected, susceptible, recovered = population.getPopulationSIR()
            # if simulation is over 'cos no infeted, this is true
            # if simulation is over because noone is susceptible, then it is also true
            # and the simulation is over in either of these cases
            recoveredAvg += recovered + infected
            print("did", i+1, "estimates, got", recoveredAvg /(i+1), "avg recovered")

        recoveredAvg = recoveredAvg / nOfEstimates
        dataFile.write(str(infectionRate) + "," + str(recoveredAvg) + "," + str(recoveryRate) + "\n")
        gammas.append(recoveryRate)
        recoveredPerGamma.append(recoveredAvg)
        recoveryRate = recoveryRate + step

    dataFile.close()




populationSize= 1000
# make it a square grid
gridSize = 100


diffusionRate = 0.8
infectionRate = 0.6
recoveryRate = 0.01
nOfStartingCases = 10

#getBetaGammaRelationship(diffusionRate, infectionRate, recoveryRate, populationSize, gridSize, nOfStartingCases)
getBetaGammaRelationship(infectionRate)


## now load data for the other graph
fil = open('./data/recoveredPerGamma.csv', 'r')
lines = fil.readlines()

gammas = []
recoveredAvgs = []

for line in lines:
    line = line.strip().split(",")
    recoveredAvgs.append(float(line[0]))
    gammas.append(float(line[1]))
fil.close()

fig, ax = plt.subplots()

ax.set_xlabel('recovery rate')
ax.set_ylabel('final number of recovered agents')
ax.set_title(r'd = ' + str(diffusionRate) + r', $\beta =$ ' + str(infectionRate) 
        + ', starting number of infected = 1\%')
plt.plot(gammas, recoveredAvgs, c="tab:red")
plt.savefig('nOfRecoveredPerGammaWithBeta' + str(infectionRate) + '.png')
plt.show()


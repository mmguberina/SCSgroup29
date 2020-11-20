from model import *
from celluloid import Camera
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt


def getBetaGammaRelationship(diffusionRate, infectionRate, recoveryRate, populationSize, gridSize, nOfStartingCases):
    gammas = []
    recoveredPerGamma = []
    step = 0.002
    recoveryRate = 0.002
    nOfEstimates = 10
    dataFile = open('./data/recoveredPerGamma.csv', 'w')
    while recoveryRate < 0.3:
        print("running estimate on recovery rate:", recoveryRate)
        recoveredAvg = 0
        for i in range(nOfEstimates):
            population = Population(diffusionRate, infectionRate, recoveryRate, populationSize, gridSize, nOfStartingCases)
            population.runDynamicsTillEnd()
            infected, susceptible, recovered = population.getPopulationSIR()
            recoveredAvg += recovered

        recoveredAvg = recoveredAvg / nOfEstimates
        dataFile.write(str(recoveredAvg) + "," + str(recoveryRate) + "\n")
        gammas.append(recoveryRate)
        recoveredPerGamma.append(recoveredAvg)
        recoveryRate += step

    dataFile.close()
    fig, ax = plt.plot()






populationSize= 1000
# make it a square grid
gridSize = 100


diffusionRate = 0.8
infectionRate = 0.6
recoveryRate = 0.01
nOfStartingCases = 10

population = Population(diffusionRate, infectionRate, recoveryRate, populationSize, gridSize, nOfStartingCases)
#population.plotPopulation()
#population.runDynamicsTillEnd()
#population.plotPopulation()

fig, ax = plt.subplots()
camera = Camera(fig)
population.animatePopulation(ax, camera)
animation = camera.animate()
animation.save('somerun.mp4')
plt.show()

# now load data for the other graph
fil = open('./data/sir_data.csv', 'r')
lines = fil.readlines()

infectedInTime = []
susceptibleInTime = []
recoveredInTime = []

x = 0
for line in lines:
    line = line.strip().split(",")
    susceptibleInTime.append([x, int(line[0])])
    infectedInTime.append([x, int(line[1])])
    recoveredInTime.append([x, int(line[2])])
    x += 1
fil.close()

infectedInTime = np.array(infectedInTime)
recoveredInTime = np.array(recoveredInTime)
susceptibleInTime = np.array(susceptibleInTime)
fig, ax = plt.subplots()

ax.set_xlabel('iteration number')
ax.set_ylabel('number of agents')
ax.set_title(r'd = ' + str(diffusionRate) + r', $\beta =$ ' + str(infectionRate) 
        + ', $\gamma =$ ' + str(recoveryRate))
plt.plot(infectedInTime[:,0], infectedInTime[:,1], c="tab:red")
plt.plot(recoveredInTime[:,0], recoveredInTime[:,1], c="tab:green")
plt.plot(susceptibleInTime[:,0], susceptibleInTime[:,1], c="tab:blue")
plt.savefig('somerun.png')
plt.show()


import numpy as np

class Warehouse:
    def __init__(self, populationSize, gridSize, expectedNOfPackages):
        self.populationSize = populationSize
        self.gridSize = gridSize
        positions = np.fix(np.random.random((self.populationSize,2)) * self.gridSize)
        # convert this into a list of tuples
        individualStates = np.zeros(self.populationSize)
        self.individuals = []
        for i in range(self.populationSize):
            self.individuals.append(Robot(positions[i], individualStates[i], gridSize))




    def 





    def checkSimulationEnd(self):
        check = 0
        infected = 0
        susceptible = 0
        recovered = 0
        for individualIndex in self.individuals:
            if self.individuals[individualIndex].individualState == 0:
                susceptible += 1
            if self.individuals[individualIndex].individualState == 1:
                infected += 1

        if susceptible == 0 or infected == 0:
            return True

        return False


    def runDynamicsTillEnd(self):
        while not self.checkSimulationEnd():
            self.infect()
            self.moveWithoutWall()
            self.recover()



    def runDynamicsOneStep(self):
        self.infect()
        self.moveWithoutWall()
        self.recover()

    def plotPopulationSnapshot(self):
        fig, ax = plt.subplots()
        susceptible = []
        infected = []
        recovered = []

        for individualIndex in self.individuals:
            if self.individuals[individualIndex].individualState == 0:
                susceptible.append(self.individuals[individualIndex].position)

            if self.individuals[individualIndex].individualState == 1:
                infected.append(self.individuals[individualIndex].position)
        
            if self.individuals[individualIndex].individualState == 2:
                recovered.append(self.individuals[individualIndex].position)

        susceptible = np.array(susceptible)
        infected = np.array(infected)
        recovered = np.array(recovered)

        if len(susceptible) > 0:
            ax.scatter(susceptible[:,0], susceptible[:,1], c='tab:blue')
        if len(infected) > 0:
            ax.scatter(infected[:,0], infected[:,1], c='tab:red')
        if len(recovered) > 0:
            ax.scatter(recovered[:,0], recovered[:,1], c='tab:green')
        
        plt.show()


       
    def animatePopulation(self, ax, camera):
        iternum = 0

        infectedInTime = []
        susceptibleInTime = []
        recoveredInTime = []

        while not self.checkSimulationEnd() and iternum < 10000:
            self.infect()
            self.moveWithoutWall()
            self.recover()
            susceptible = []
            infected = []
            recovered = []

            for individualIndex in self.individuals:
                if self.individuals[individualIndex].individualState == 0:
                    susceptible.append(self.individuals[individualIndex].position)

                if self.individuals[individualIndex].individualState == 1:
                    infected.append(self.individuals[individualIndex].position)
            
                if self.individuals[individualIndex].individualState == 2:
                    recovered.append(self.individuals[individualIndex].position)

            infectedInTime.append(len(infected))
            susceptibleInTime.append(len(susceptible))
            recoveredInTime.append(len(recovered))

            if iternum % 20 == 0:
                susceptible = np.array(susceptible)
                infected = np.array(infected)
                recovered = np.array(recovered)

                if len(susceptible) > 0:
                    ax.scatter(susceptible[:,0], susceptible[:,1], c='tab:blue')
                if len(infected) > 0:
                    ax.scatter(infected[:,0], infected[:,1], c='tab:red')
                if len(recovered) > 0:
                    ax.scatter(recovered[:,0], recovered[:,1], c='tab:green')

                camera.snap()
            iternum += 1


#### implement genetic algorithm
## to figure out:
# loop a certain amount of generations?
# what seletion?

import numpy as np

# init
def geneticAlgo(chromosomeLength, populationSize, generations, mutationParameter, selectionParameter):
    m = chromosomeLength
    n = populationSize
    population = np.random.randint(0, 2, size=(m,n))
    tempPopulation = np.array(population)
    percentMutated = np.zeros((n,))
    fitness = np.zeros((n,))
    maxfitness = 0
    bestIndividual = np.zeros((m,))

    for i in range(n):

        currIndividual = population[:,i]
        fitness[i] = evaluateIndividual(currIndividual)
        if fitness[i] > maxfitness:
            maxfitness = fitness[i]
            bestIndividual = np.array(currIndividual)
    print("Initial best: ")
    print(str(bestIndividual) + ", with fitness " + str(maxfitness))
    for generation in range(generations):
        tempPopulation = np.array(population)

        # evaluate individuals
        for i in range(n):
            fitness[i] = evaluateIndividual(population[:,i])
            if fitness[i] > maxfitness:
                print("At generation " + str(generation) + ", found new optimal candidate:")
                maxfitness = fitness[i]
                bestIndividual = population[:,i]
                print(str(bestIndividual) + ", with fitness " + str(maxfitness))

        for i in range(n):
            # tournament selection
            ind1 = selectIndividual(fitness, selectionParameter)
            ind2 = selectIndividual(fitness, selectionParameter)

            # crossover
            tempPopulation[:,i] = crossover(population[:,ind1], population[:,ind2])

        # mutate
        for i in range(n):
            tempPopulation[:,i] = mutate(tempPopulation[:,i], mutationParameter)

        # elitism
        tempPopulation[:,0] = bestIndividual
        population = np.array(tempPopulation)

    return(population[:,0])



def evaluateIndividual(individual):
    # i.e. decode individual into parameter values, call robot roam with the values
    decodedIndividual = decode(individual)

    crossPoint = int(np.floor(len(individual)/2))
    fitness = sum(individual[:crossPoint]) - sum(individual[crossPoint:])
    return(fitness)

def decode(individual):
    # TODO: figure out encoding
    # inputs to activeSwimmers:
    # x, y, fi, item_positions_set, delivery_station, n, dt, T0, nOfRobots, ni, v,
    # trans_dif_T, rot_dif_T, gridSize, particle_radius, torque_radius, out=None

    # probably parameters for attraction-repulsion
    # when to change between states "random-walk", "go-straight-to-item", "stuck"
    return(0)

def mutate(individual, mutationParameter):
    m = len(individual)
    toMutate = np.where(np.random.rand(m) < mutationParameter)
    mutated = np.array(individual)
    mutated[toMutate] = 1 - mutated[toMutate]
    testMut = np.zeros(individual.shape)
    for i in range(len(individual)):
        testMut[i] = individual[i]
    return(mutated)

def crossover(individual1, individual2):
    crossOver = np.random.randint(len(individual1))
    newIndividual = np.hstack((individual1[:crossOver], individual2[crossOver:]))
    return(newIndividual)


def selectIndividual(fitness, selectionParameter):
    # returns index of fitter individual with probability selectionParameter, else returns other
    n = len(fitness)
    i1 = np.random.randint(n)
    i2 = np.random.randint(n)

    r = np.random.rand()
    if r > selectionParameter:
        if fitness[i1] > fitness[i2]:
            return(i1)
        else:
            return(i2)
    else:
        if fitness[i1] > fitness[i2]:
            return(i2)
        else:
            return(i1)


def testing():
    mx = geneticAlgo(200, 400, 5000, 0.05, 0.8)
    print("Maximum found with chromosome:    " + str(mx))
    print("Giving function value: " + str(evaluateIndividual(mx)))



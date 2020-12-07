import numpy as np

def geneticAlgo(chromosomeLength, populationSize, generations, mutationParameter, selectionParameter):
    # some initializations
    m = chromosomeLength
    n = populationSize
    population = np.random.randint(0, 2, size=(m,n))
    tempPopulation = np.array(population)
    percentMutated = np.zeros((n,))
    fitness = np.zeros((n,))
    maxfitness = 0
    bestIndividual = np.zeros((m,))

    # find best individual in initial population
    for i in range(n):
        currIndividual = population[:,i]
        fitness[i] = evaluateIndividual(currIndividual)
        if fitness[i] > maxfitness:
            maxfitness = fitness[i]
            bestIndividual = np.array(currIndividual)

    print("Initial best: ")
    print(str(bestIndividual) + ", with fitness " + str(maxfitness))


    for generation in range(generations):
        # reinitialize with np.array since I had a problem where they treated tempPop and pop as the same object, dunno if we need to do this everywhere?
        tempPopulation = np.array(population)

        # evaluate individuals, find best
        for i in range(n):
            fitness[i] = evaluateIndividual(population[:,i])
            if fitness[i] > maxfitness:
                print("At generation " + str(generation) + ", found new optimal candidate:")
                maxfitness = fitness[i]
                bestIndividual = population[:,i]
                print(str(bestIndividual) + ", with fitness " + str(maxfitness))

        # tournament selection + crossover
        for i in range(n):
            ind1 = selectIndividual(fitness, selectionParameter)
            ind2 = selectIndividual(fitness, selectionParameter)
            tempPopulation[:,i] = crossover(population[:,ind1], population[:,ind2])

        # mutate
        for i in range(n):
            tempPopulation[:,i] = mutate(tempPopulation[:,i], mutationParameter)

        # elitism
        tempPopulation[:,0] = bestIndividual
        population = np.array(tempPopulation)

    # due to elitism, the first object is always the best
    return(population[:,0])



def evaluateIndividual(individual):
    # TODO: figure out actual function to use
    # decode individual into parameter values, call robot roam with the values
    decodedIndividual = decode(individual)

    # currently got a random temporary function
    crossPoint = int(np.floor(len(individual)/2))
    fitness = sum(individual[:crossPoint]) - sum(individual[crossPoint:])
    return(fitness)

def decode(individual):
    # TODO: figure out encoding
    # probably parameters for attraction-repulsion
    # when to change between states "random-walk", "go-straight-to-item", "stuck"
    return(0)

def mutate(individual, mutationParameter):
    # mutates the individual with probability mutationparameter
    m = len(individual)

    toMutate = np.where(np.random.rand(m) < mutationParameter)

    mutated = np.array(individual)
    mutated[toMutate] = 1 - mutated[toMutate] # assuming binary encoding
    return(mutated)

def crossover(individual1, individual2):
    # creates new individual from the two individuals. this is done by simple concatination of two lists, with crossoverpoint chosen randomly
    crossOver = np.random.randint(len(individual1))

    # hstack handles the concatination
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



from deap import base, creator, tools, algorithms
import multiprocessing
import numpy as np
import random
import yaml
from robot_world import test_pid_parameter
from sklearn.kernel_ridge import KernelRidge

NGEN = 50 # Number of Generation
MU = 100 # Number of individual in population
CXPB = 0.8 #Crossover probability
NDIM = 24 # Number of dimension of the individual (=number of gene)
with open('car_world.yaml') as f:
    PARM = yaml.safe_load(f)
lock = multiprocessing.Lock()

# Bounds on the first 3 genes
LOW1, UP1 = 0 , 4
LOW2, UP2 = 0 , 1

BOUNDS = [(LOW2, UP2) for i in range(NDIM)]
for i in range(8):
    BOUNDS[i*3] = (LOW1,UP1)


toolbox = base.Toolbox()

def init_opti():
    creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0))
    creator.create("Individual", list, typecode='f', fitness=creator.FitnessMin)
    
    toolbox.register("individual", init_ind, icls=creator.Individual, ranges=BOUNDS)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("evaluate", evaluation)
    toolbox.register("mate", tools.cxOnePoint)
    toolbox.register("mutate", tools.mutGaussian, mu=[2,0.5,0.5], sigma=[0.5,0.1,0.1], indpb=0.1)
    toolbox.register("select", tools.selNSGA2)
    
def init_ind(icls, ranges):
    genome = list()

    for p in ranges:
        genome.append(np.random.rand()*(p[1]-p[0])+p[0])
    return icls(genome)

def evaluation(ind):
    # lock.acquire()
    # try:
    X = np.arange(0.5,4,1).reshape(-1,1)
    y = ind.reshape(-1,6)
    model = KernelRidge(alpha=1.0)
    model.fit(X,y)
    objective1,objective2 = test_pid_parameter(model=model)
    # finally:
    #     lock.release()
    
    return objective1, objective2

def main():
    pareto = tools.ParetoFront()

    pop = toolbox.population(n=MU)
    graph = []
    data = []

    print("init:")
    # Evaluate the individuals with an invalid fitness
    invalid_ind = [ind for ind in pop if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
    data.append(fitnesses)
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit
        graph.append(ind.fitness.values)

    # This is just to assign the crowding distance to the individuals
    # no actual selection is done
    pop = toolbox.select(pop, len(pop))

    # Begin the generational process
    for gen in range(1, NGEN):
        print("Generation: {}".format(gen))
        # Vary the population
        offspring = tools.selTournamentDCD(pop, len(pop))
        offspring = [toolbox.clone(ind) for ind in offspring]

        for ind1, ind2 in zip(offspring[::2], offspring[1::2]):
            if random.random() <= CXPB:
                toolbox.mate(ind1, ind2)
                
            toolbox.mutate(ind1)
            toolbox.mutate(ind2)
            del ind1.fitness.values, ind2.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        data.append(fitnesses)
        
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
            graph.append(ind.fitness.values)

        # Select the next generation population
        pop = toolbox.select(pop + offspring, MU)

    pareto.update(pop)

    return pop, pareto, graph, data

if __name__ == "__main__":
    init_opti()
#    Multiprocessing pool: I want to compute faster
    # pool = multiprocessing.Pool()
    # toolbox.register("map", pool.map)
    
    pop, optimal_front, graph_data, data = main()

    # Saving the Pareto Front, for further exploitation
    with open('./pareto_front.txt', 'w') as front:
        for ind in optimal_front:
            front.write(str(ind) + '\n')

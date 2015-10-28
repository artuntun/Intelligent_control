__author__ = 'ARturo'
## TUTORIAL from http://lethain.com/genetic-algorithms-cool-name-damn-simple/
# genetic algorithm to find combinations of 5 numbers which sum 200

from random import randint
def individual(length, min, max):
    'Create a memeber of the population'
    return [randint(min, max) for x in xrange(length)]

def population(count, length, min, max):
    """
    Create a number of individuals ( i.e. a population)
    :param count: size of population
    :param length: size of each individual
    :param min: minimum for each of the values in the individual's list
    :param max: maximum for each of the values in the individual's list
    :return: a list of individuals
    """

    return [individual(length, min, max) for x in xrange(count)]

from operator import add

def fitness(individual, target):
    """
    Define how good a individual is. The lower, the better.

    :param individual: individual we are evaluating
    :param target: the sum we are aiming
    :return: distance to target
    """

    sum = reduce(add, individual, 0)
    return abs(target-sum)

def grade(pop, target):
    'Avarage fitness for a population'
    summed = reduce(add, (fitness(x, target) for x in pop), 0)
    return summed/ (len(pop) * 1.0)

from random import random

def evolve(pop, target, retain = 0.2, random_select = 0.05, mutate = 0.01):
    graded = [(fitness(x, target), x) for x in pop] # return individuals with the fitness
    graded = [x[1] for x in sorted(graded)] # return only individuals ordered by fitness
    retain_length = int(len(pop) * retain)
    parents = graded[: retain_length] #the parents are the 20percent better

    #randomly add other individuals to promote  genetic diveristy
    for individual in graded[retain_length:]:
        if random_select > random():
            parents.append(individual)

    #mutate some individuals
    for individual in parents:
        if mutate > random():
            pos_to_mutate = randint(0, len(individual)-1)
            # this mutation is not ideal, because it
            # restricts the range of possible values,
            # but the function is unaware of the min/max
            # values used to create the individuals,
            individual[pos_to_mutate] = randint(
                min(individual), max(individual))

    # crossover parents to create children
    parents_length = len(parents)
    desired_length = len(pop) - parents_length
    children = []
    while len(children) < desired_length:
        male = randint(0, parents_length-1)
        female = randint(0, parents_length-1)
        if male != female:
            male = parents[male]
            female = parents[female]
            half = len(male) / 2
            child = male[:half] + female[half:]
            children.append(child)

    parents.extend(children)
    return parents

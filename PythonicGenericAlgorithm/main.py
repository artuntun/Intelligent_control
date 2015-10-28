__author__ = 'ARturo'

import genetic as gn

target = 371
p_count = 100
i_length = 5
i_min = 0
i_max = 100
p = gn.population(p_count, i_length, i_min, i_max)
fitness_history = [gn.grade(p, target),]
for x in xrange(100):
    p = gn.evolve(p, target)
    fitness_history.append(gn.grade(p, target))


from operator import add
print reduce(add, p[1], 0)
print reduce(add, p[2], 0)
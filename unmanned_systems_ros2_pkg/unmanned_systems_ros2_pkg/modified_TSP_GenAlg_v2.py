#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 17 13:17:33 2021

@author: fieldstd
"""


import numpy as np
import matplotlib.pyplot as plt
import time
import pylab as pl # Needed for plotting numbers on plots
#%matplotlib inline
#%config InlineBackend.figure_format = 'svg'
plt.style.use("seaborn")
np.random.seed(42)

class path_store:
    def __init__(self, x, y, cost):
        self.x = x
        self.y = y
        self.cost = cost

# Traveling Salesman Problem Example #
#       5 Desired Waypoints

# my_code...
# from Scenario import Scenario
scenario: Scenario = Scenario().loader(
    #####  the naming structure for scenarios  #####
    # "scenarios / SearchType _ grid-size _ bot-size _ grid-spacing.json"
    # "scenarios/AStar_10x10_bot-0o5_grid-0o5.json"  --> -->
    # "scenarios / AStar _ 10x10 grid _ bot radius 0.5 _ grid spacing 0.5"
    # if 'random' is at the end, then random start/goal pos and/or obstacle pos
    #####

    #####  scenarios in the "scenarios" folder  #####
    # "scenarios/AStar_10x10_bot-0o5_grid-0o5.json"
    # "scenarios/AStar_10x10_bot-0o5_grid-0o5_random.json"
    # "scenarios/AStar_15x15_bot-0o5_grid-0o5a.json"       # Exam2 Problem 3
    "scenarios/AStar_15x15_bot-0o5_grid-0o5b.json"       # Exam2 Problem 4
    # "scenarios/AStar_15x15_bot-0o5_grid-1o0.json"        # HW5 problem 1a
    # "scenarios/AStar_50x50_bot-0o5_grid-0o5.json"        # HW3 problem 2
    # "scenarios/AStar_50x50_bot-0o5_grid-0o5_random.json"
    # "scenarios/Dijkstra_15x15_bot-0o5_grid-1o0.json"     # HW5 problem 1b
    # "scenarios/RRT_10x10_bot-0o5_grid-0o5.json"
    # "scenarios/RRT_10x10_bot-0o5_grid-0o5_random.json"
    # "scenarios/RRT_15x15_bot-0o5_grid-1o0.json"          # HW5 problem 1c
    # "scenarios/RRT_50x50_bot-0o5_grid-0o5.json"          # HW3 problem 3
    #####
)

# from Node import Node
# nodes = [
#     Node(1,1),
#     Node(9,7),
#     Node(1,9),
#     Node(4,4),
#     Node(9,4),
#     Node(6,14),
#     Node(3,11),
#     Node(14,1),
#     Node(1,14),
#     Node(14,14),
#     Node(7,10)
# ]

##### convert my outputs to this guys inputs

# his
gx = [0, 1, 2, 9, 9] # 5 desired waypoints
gy = [0, 9, 5, 9, 2]

gx = []
gy = []

# mine
# for node in nodes:
#     gx.append(node.x)
#     gy.append(node.y)

# his
sx = 0 # Starting point
sy = 0

# # mine
# sx = nodes[0].x
# sy = nodes[0].y

# his
ox = [2, 3, 4, 4, 4, 4, 10, 9, 8, 7, 6, 6]
oy = [7, 7, 7, 8, 9, 10, 5, 5, 5, 5, 5, 4]

ox = []
oy = []

# mine
for obstacle in scenario.obstacles.values():
    ox.append(obstacle.x)
    oy.append(obstacle.y)
      
# his
grid_size = 0.5
grid_x = 10
grid_y = 10
min_x = 0
min_y = 0

# mine
grid_size = scenario.grid.grid_spacing
grid_x = scenario.grid.max_x
grid_y = scenario.grid.max_y
min_x = scenario.grid.min_x
min_y = scenario.grid.min_y

# Plotting   
plt.plot(ox, oy, ".b")
plt.plot(sx, sy, "xg")
plt.plot(gx, gy, "xr")
plt.axis([min_x, grid_x, min_y, grid_y])
# plt.show()

# his
cities = [0, 1, 2, 3, 4]

# mine
cities = list(range(1,len(nodes)))

cost_matrix = np.zeros([len(gx), len(gx)])
path_matrix = dict()

for i, start in enumerate(nodes):
    for j, goal in enumerate(nodes):
        if i == j:
            continue
        scenario.algorithm.reset()
        start.reset()
        goal.reset()
        scenario.algorithm.start = start
        scenario.algorithm.goal = goal
        scenario.algorithm.find_path()
        path_matrix[i, j] = path_store(
            [n.x for n in scenario.algorithm.path],
            [n.y for n in scenario.algorithm.path],
            scenario.algorithm.path[0].total_cost
        )
        cost_matrix[i, j] = scenario.algorithm.path[0].total_cost

adjacency_mat = cost_matrix

class Population():
    def __init__(self, bag, adjacency_mat):
        self.bag = bag
        self.parents = []
        self.score = 0
        self.best = None
        self.adjacency_mat = adjacency_mat

def init_population(cities, adjacency_mat, n_population):
    return Population(
        np.asarray([np.random.permutation(cities) for _ in range(n_population)]), 
        adjacency_mat
    )

pop = init_population(cities, adjacency_mat, 5)

def fitness(self, chromosome):
    return (sum([
        self.adjacency_mat[chromosome[i], chromosome[i + 1]]
        for i in range(len(chromosome) - 1)]) 
        + self.adjacency_mat[0, chromosome[0]])  # because we have to start at first node every time

Population.fitness = fitness

def evaluate(self):
    distances = np.asarray(
        [self.fitness(chromosome) for chromosome in self.bag]
    )
    self.score = np.min(distances)
    self.best = self.bag[distances.tolist().index(self.score)]
    self.parents.append(self.best)
    if False in (distances[0] == distances):
        distances = np.max(distances) - distances
    return distances / np.sum(distances)
    
Population.evaluate = evaluate

def select(self, k=4):
    fit = self.evaluate()
    while len(self.parents) < k:
        idx = np.random.randint(0, len(fit))
        if fit[idx] > np.random.rand():
            self.parents.append(self.bag[idx])
    self.parents = np.asarray(self.parents)

Population.select = select

def swap(chromosome):
    a, b = np.random.choice(len(chromosome), 2)
    chromosome[a], chromosome[b] = (
        chromosome[b],
        chromosome[a],
    )
    return chromosome

def crossover(self, p_cross=0.1):
    children = []
    count, size = self.parents.shape
    for _ in range(len(self.bag)):
        if np.random.rand() > p_cross:
            children.append(
                list(self.parents[np.random.randint(count, size=1)[0]])
            )
        else:
            parent1, parent2 = self.parents[
                np.random.randint(count, size=2), :
            ]
            idx = np.random.choice(range(size), size=2, replace=False)
            start, end = min(idx), max(idx)
            child = [None] * size
            for i in range(start, end + 1, 1):
                child[i] = parent1[i]
            pointer = 0
            for i in range(size):
                if child[i] is None:
                    while parent2[pointer] in child:
                        pointer += 1
                    child[i] = parent2[pointer]
            children.append(child)
    return children

Population.crossover = crossover

def mutate(self, p_cross=0.1, p_mut=0.1):
    next_bag = []
    children = self.crossover(p_cross)
    for child in children:
        if np.random.rand() < p_mut:
            next_bag.append(swap(child))
        else:
            next_bag.append(child)
    return next_bag
    
Population.mutate = mutate



def genetic_algorithm(
    cities,
    adjacency_mat,
    n_population=500,
    n_iter=2000,
    selectivity=0.75,
    p_cross=0.5,
    p_mut=0.3,
    print_interval=100,
    return_history=False,
    verbose=False,
):
    pop = init_population(cities, adjacency_mat, n_population)
    best = pop.best
    score = float("inf")
    history = []
    for i in range(n_iter):
        pop.select(n_population * selectivity)
        history.append(pop.score)
        if verbose:
            print(f"Generation {i}: {pop.score}")
        elif i % print_interval == 0:
            print(f"Generation {i}: {pop.score}")
        if pop.score < score:
            best = pop.best
            score = pop.score
        children = pop.mutate(p_cross, p_mut)
        pop = Population(children, pop.adjacency_mat)
    if return_history:
        return best, history
    return best

scenario.algorithm.stopwatch.start()
best = genetic_algorithm(cities, adjacency_mat, verbose=True)
best.insert(0, 0)
scenario.algorithm.stopwatch.stop()
print(f"Time: {scenario.algorithm.stopwatch.elapsed_time}")

for i, n in enumerate(best):
    plt.text(nodes[n].x, nodes[n].y, str(i))

# Plotting   
plt.plot(ox, oy, ".b")
plt.plot(sx, sy, "xg")
plt.plot(gx, gy, "xr")
plt.axis([min_x-grid_size, grid_x+grid_size, min_y-grid_size, grid_y+grid_size])

for i in range(1,len(best)):
    plt.plot(path_matrix[best[i-1],best[i]].x,path_matrix[best[i-1],best[i]].y,'r-')
#plt.plot(pathx, pathy, "-r")
plt.xlabel('X Distance')
plt.ylabel('Y Distance')
plt.show()






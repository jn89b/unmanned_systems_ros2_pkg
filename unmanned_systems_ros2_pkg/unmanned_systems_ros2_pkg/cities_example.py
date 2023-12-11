from itertools import permutations
import math as m
import time 

waypoints = [(1,1), (9,7), (1,9), (4,4), (9,4), 
             (6,14), (3,11), (14,1), 
             (1,14), (14,14), (7,10)]

other_waypoints = {}
cost_dictionary = {}

#Compute all the costs from some waypoint to all other waypoints
compute_times = 0
for wp in waypoints: 
    for other_wp in waypoints:
        if wp == other_wp:
            continue
        if (wp, other_wp) in cost_dictionary:
            continue
        if (other_wp, wp) in cost_dictionary:
            continue
        else:
            compute_times += 1
            ## this would be where you plug in astar
            #waypoints = call_astar
            #total distance = sum of all the distances in the path
            total_distance = m.dist(wp, other_wp)
            #a->c
            cost_dictionary[wp, other_wp] = total_distance
            #c->a 
            cost_dictionary[other_wp, wp] = total_distance

#%%

# Compute all the possible paths
paths = list(permutations(waypoints, len(waypoints)))
total_cost = []
print("Number of compute times: ", compute_times)
print("Number of paths: ", len(paths))


#%%
n_iterations = 100

start_time = time.time()
for j, path in enumerate(paths):    
    sum_cost = 0
    if j % n_iterations == 0:
        print("Iteration: ", j)
    
    for i in range(len(path)-1):
        sum_cost += cost_dictionary[path[i], path[i+1]]
        
    total_cost.append(sum_cost)
        
end_time = time.time()
print("Time elapsed: ", end_time - start_time)
# get best path
min_total_cost = min(total_cost)
min_total_cost_index = total_cost.index(min_total_cost)
best_path = paths[min_total_cost_index]


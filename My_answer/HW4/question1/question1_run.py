from mdp import mdp
import numpy as np
from grid_map import grid_map
from mdp import mdp
from ir_sim.env import env_base
from pathlib import Path

map_matrix = np.load('intelligent-robot-simulator/Homework/HW4/question1/map_matrix.npy')
reward_matrix = np.load('intelligent-robot-simulator/Homework/HW4/question1/reward_matrix.npy')

env = env_base(world_width = 20, world_height = 20)
grid_map = grid_map(map_matrix=map_matrix, reward_matrix=reward_matrix)
mdp = mdp(grid_map)

grid_map.show_map()

## please complete the function value_iteration()
policy_value = mdp.value_iteration()
# print(policy_value)
for i in range(20):
    for j in range(20):
        print(i,j,policy_value[i][j])









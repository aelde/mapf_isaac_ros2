import heapq
import numpy as np
{'loc': [(1, 6)], 'g_val': 8, 'h_val': 0, 'parent': {'loc': [(1, 5)], 'g_val': 7, 'h_val': 1, 'parent': {'loc': [(1, 4)], 'g_val': 6, 'h_val': 2, 'parent': {'loc': [(1, 3)], 'g_val': 5, 'h_val': 3, 'parent': {'loc': [(1, 2)], 'g_val': 4, 'h_val': 4, 'parent': {'loc': [(2, 2)], 'g_val': 3, 'h_val': 5, 'parent': {'loc': [(3, 2)], 'g_val': 2, 'h_val': 6, 'parent': {'loc': [(4, 2)], 'g_val': 1, 'h_val': 7, 'parent': {
    'loc': [(5, 2)], 'g_val': 0, 'h_val': 8, 'parent': None, 'timestep': 0, 'reached_goal': [False]}, 'timestep': 1, 'reached_goal': [False]}, 'timestep': 2, 'reached_goal': [False]}, 'timestep': 3, 'reached_goal': [False]}, 'timestep': 4, 'reached_goal': [False]}, 'timestep': 5, 'reached_goal': [False]}, 'timestep': 6, 'reached_goal': [False]}, 'timestep': 7, 'reached_goal': [False]}, 'timestep': 8, 'reached_goal': [True]}


parent = np.array([2, 2])
curr = np.array([1, 2])
child = np.array([1, 3])
# curr_1 = np.array([1,2])
# child_2 = np.array([3,2])
# child_1 = np.array([1,3])


# new_direction_str_1 = np.array([0,2]) - np.array(child)
# new_direction_str_2 = np.array(child_2) - np.array(curr)
# new_direction_rot = np.array(child_1) - np.array(curr_1)

direc = curr - parent
new_direction_str = np.array(child) - np.array(curr)

rotate_left_cost = 'left'
rotate_right_cost = 'right'

print(f'new_d_str: {new_direction_str}')
# print(f'new_d_str_1: {new_direction_str_1}')
# print(f'new_d_str_2: {new_direction_str_2}')
# print(f'new_d_rot: {new_direction_rot}')
print(f'direc: {direc}')

print(f'np.zero2: {np.zeros(2)}')
needs_rotation = not np.array_equal(
    direc, new_direction_str) and not np.array_equal(direc, np.zeros(2))
if needs_rotation:
    cross_product = np.cross(direc, new_direction_str)
    print(f'cross_product: {cross_product}')
    rotation_cost = rotate_left_cost if cross_product > 0 else rotate_right_cost
    #tentative_g_score = g_score[current] + rotation_cost
    print(f"need rotation! {rotation_cost}")
else : 
	print("no need rotation!")
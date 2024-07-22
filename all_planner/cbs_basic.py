import time as timer
import heapq
import random
import matplotlib.pyplot as plt
from a_star_class import A_Star, get_location, get_sum_of_cost, compute_heuristics
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.colors import to_rgba
import os
from all_planner.DICISION import DIR_COST

def each_astar_step(my_map, starts, goals, heuristics, all_paths, constraints, step):
    n_x = len(my_map)
    n_y = len(my_map[0])
    num_agents = len(starts)
    
    colors = ['lime', 'deeppink', 'blue']
    
    for agent in range(num_agents):
        fig, ax = plt.subplots(figsize=(8, 8))
        fig.suptitle(f'Agent {agent} - Step {step}', fontsize=16)
        
        # Create a heatmap of the map (obstacles)
        ax.imshow(my_map, cmap='binary')
        
        # Plot heuristic values for this agent
        for x in range(n_x):
            for y in range(n_y):
                if (x, y) in heuristics[agent]:
                    h_value = heuristics[agent][(x, y)]
                    ax.text(y, x, f'{h_value}', ha='center', va='center', fontsize=6, 
                            color='red' if my_map[x][y] else 'black')
        
        # Plot constraints for this agent
        for constraint in constraints:
            if constraint['timestep'] == step and constraint['agent'] == agent:
                if len(constraint['loc']) == 1:
                    ax.add_patch(plt.Circle((constraint['loc'][0][1], constraint['loc'][0][0]), 0.3, color='red', fill=False))
                else:
                    ax.add_patch(plt.Rectangle((min(constraint['loc'][0][1], constraint['loc'][1][1]) - 0.5, 
                                                min(constraint['loc'][0][0], constraint['loc'][1][0]) - 0.5), 
                                               1, 1, color='red', fill=False))
        
        # Plot all paths explored by this agent
        for i, paths in enumerate(all_paths):
            if agent < len(paths) and paths[agent]:  # Check if path exists for this agent
                path = paths[agent]
                if step < len(path):
                    current_pos = path[step]
                    path_color = to_rgba(colors[agent], alpha=0.1 + 0.8 * (i / len(all_paths)))
                    
                    # Plot path up to current step
                    path_x = [pos[1] for pos in path[:step+1]]
                    path_y = [pos[0] for pos in path[:step+1]]
                    ax.plot(path_x, path_y, '-', color=path_color, linewidth=2)
                    
                    # Plot current position for the most recent path
                    if i == len(all_paths) - 1:
                        ax.plot(current_pos[1], current_pos[0], 'o', color=colors[agent], markersize=10, label=f'Current Position')
        
        # Plot start and goal positions
        ax.plot(starts[agent][1], starts[agent][0], 's', color=colors[agent], markersize=8, markerfacecolor='none', label='Start')
        ax.plot(goals[agent][1], goals[agent][0], '*', color=colors[agent], markersize=10, label='Goal')
        
        # Set major ticks
        ax.set_xticks(np.arange(0, n_y, 1))
        ax.set_yticks(np.arange(0, n_x, 1))
        
        # Label major ticks
        ax.set_xticklabels(np.arange(0, n_y, 1))
        ax.set_yticklabels(np.arange(0, n_x, 1))
        
        # Add gridlines
        ax.grid(which='major', color='gray', linestyle='-', linewidth=0.5)
        
        # Set labels
        ax.set_xlabel('Y')
        ax.set_ylabel('X')
        
        # Add legend
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=2)
        
        # Save the figure
        os.makedirs(f'result/agent_{agent}', exist_ok=True)
        plt.savefig(f'result/agent_{agent}/step_{step:03d}.png', dpi=300, bbox_inches='tight')
        plt.close(fig)

def astar_step(my_map, starts, goals, heuristics, paths, constraints, step):
    n_x = len(my_map)
    n_y = len(my_map[0])
    num_agents = len(starts)
    
    fig, ax = plt.subplots(figsize=(10, 10))
    fig.suptitle(f'A* Step {step}', fontsize=16)
    
    colors = ['lime', 'deeppink', 'blue']
    
    # Create a heatmap of the map (obstacles)
    ax.imshow(my_map, cmap='binary')
    
    # Plot heuristic values
    for x in range(n_x):
        for y in range(n_y):
            for i in range(num_agents):
                if (x, y) in heuristics[i]:
                    h_value = heuristics[i][(x, y)]
                    ax.text(y, x, f'{h_value}', ha='center', va='center', fontsize=6, 
                            color='red' if my_map[x][y] else 'black')
    
    # Plot constraints
    for constraint in constraints:
        if constraint['timestep'] == step:
            if len(constraint['loc']) == 1:
                ax.add_patch(plt.Circle((constraint['loc'][0][1], constraint['loc'][0][0]), 0.3, color='red', fill=False))
            else:
                ax.add_patch(plt.Rectangle((min(constraint['loc'][0][1], constraint['loc'][1][1]) - 0.5, 
                                            min(constraint['loc'][0][0], constraint['loc'][1][0]) - 0.5), 
                                           1, 1, color='red', fill=False))
    
    # Plot current positions and paths
    for i in range(num_agents):
        if step < len(paths[i]):
            current_pos = paths[i][step]
            ax.plot(current_pos[1], current_pos[0], 'o', color=colors[i], markersize=10, label=f'Agent {i}')
            
            # Plot path up to current step
            path_x = [pos[1] for pos in paths[i][:step+1]]
            path_y = [pos[0] for pos in paths[i][:step+1]]
            ax.plot(path_x, path_y, '-', color=colors[i], linewidth=2, alpha=0.5)
        
        # Plot start and goal positions
        ax.plot(starts[i][1], starts[i][0], 's', color=colors[i], markersize=8, markerfacecolor='none')
        ax.plot(goals[i][1], goals[i][0], '*', color=colors[i], markersize=10)
    
    # Set major ticks
    ax.set_xticks(np.arange(0, n_y, 1))
    ax.set_yticks(np.arange(0, n_x, 1))
    
    # Label major ticks
    ax.set_xticklabels(np.arange(0, n_y, 1))
    ax.set_yticklabels(np.arange(0, n_x, 1))
    
    # Add gridlines
    ax.grid(which='major', color='gray', linestyle='-', linewidth=0.5)
    
    # Set labels
    ax.set_xlabel('Y')
    ax.set_ylabel('X')
    
    # Add legend
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=3)
    
    # Save the figure
    os.makedirs('result', exist_ok=True)
    plt.savefig(f'result/step_{step:03d}.png', dpi=300, bbox_inches='tight')
    plt.close(fig)

def visualize_map_and_heuristics(my_map, starts, goals, heuristics):
    n_x = len(my_map)
    n_y = len(my_map[0])
    num_agents = len(starts)
    
    fig, axs = plt.subplots(1, num_agents, figsize=(6*num_agents, 6), squeeze=False)
    fig.suptitle('Map Visualization with Start, Goal Positions, and Heuristics', fontsize=16)
    
    colors = ['lime', 'deeppink', 'blue','cyan']
    
    for i in range(num_agents):
        ax = axs[0, i]
        
        # Create a heatmap of the map (obstacles)
        ax.imshow(my_map, cmap='binary')
        
        # Plot heuristic values
        for x in range(n_x):
            for y in range(n_y):
                if (x, y) in heuristics[i]:
                    h_value = heuristics[i][(x, y)]
                    ax.text(y, x, f'{h_value}', ha='center', va='center', fontsize=6, 
                            color='red' if my_map[x][y] else 'black')
        
        # Plot start position
        ax.plot(starts[i][1], starts[i][0], 'o', color=colors[i], markersize=10, label=f'Start {i}')
        
        # Plot goal position
        ax.plot(goals[i][1], goals[i][0], '*', color=colors[i], markersize=10, label=f'Goal {i}')
        
        # Set major ticks
        ax.set_xticks(np.arange(0, n_y, 1))
        ax.set_yticks(np.arange(0, n_x, 1))
        
        # Label major ticks
        ax.set_xticklabels(np.arange(0, n_y, 1))
        ax.set_yticklabels(np.arange(0, n_x, 1))
        
        # Add gridlines
        ax.grid(which='major', color='gray', linestyle='-', linewidth=0.5)
        
        # Set labels and title
        ax.set_xlabel('Y')
        ax.set_ylabel('X')
        ax.set_title(f'Agent {i}')
        
        # Invert y-axis to match the coordinate system in the code
        # ax.invert_yaxis()
        
        # Add legend
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15))
        
    plt.tight_layout()
    plt.show()

def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    t_range = max(len(path1), len(path2))
    for t in range(t_range):
        loc_c1 = get_location(path1, t)
        loc_c2 = get_location(path2, t)
        loc1 = get_location(path1, t+1)
        loc2 = get_location(path2, t+1)
        # vertex collision
        if loc1 == loc2:
            return [loc1], t
        # edge collision
        if [loc_c1, loc1] == [loc2, loc_c2]:
            return [loc2, loc_c2], t

    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths)-1):
        for j in range(i+1, len(paths)):
            if detect_collision(paths[i], paths[j]) != None:
                position, t = detect_collision(paths[i], paths[j])
                collisions.append({'a1': i,
                                   'a2': j,
                                   'loc': position,
                                   'timestep': t+1})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    if len(collision['loc']) == 1:
        constraints.append({'agent': collision['a1'],
                            'loc': collision['loc'],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
        constraints.append({'agent': collision['a2'],
                            'loc': collision['loc'],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
    else:
        constraints.append({'agent': collision['a1'],
                            'loc': [collision['loc'][0], collision['loc'][1]],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
        constraints.append({'agent': collision['a2'],
                            'loc': [collision['loc'][1], collision['loc'][0]],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    constraints = []
    agent = random.randint(0, 1)
    a = 'a'+str(agent + 1)
    if len(collision['loc']) == 1:
        constraints.append({'agent': collision[a],
                            'loc': collision['loc'],
                            'timestep': collision['timestep'],
                            'positive': True
                            })
        constraints.append({'agent': collision[a],
                            'loc': collision['loc'],
                            'timestep': collision['timestep'],
                            'positive': False
                            })
    else:
        if agent == 0:
            constraints.append({'agent': collision[a],
                                'loc': [collision['loc'][0], collision['loc'][1]],
                                'timestep': collision['timestep'],
                                'positive': True
                                })
            constraints.append({'agent': collision[a],
                                'loc': [collision['loc'][0], collision['loc'][1]],
                                'timestep': collision['timestep'],
                                'positive': False
                                })
        else:
            constraints.append({'agent': collision[a],
                                'loc': [collision['loc'][1], collision['loc'][0]],
                                'timestep': collision['timestep'],
                                'positive': True
                                })
            constraints.append({'agent': collision[a],
                                'loc': [collision['loc'][1], collision['loc'][0]],
                                'timestep': collision['timestep'],
                                'positive': False
                                })
    return constraints


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    
        # visualize_map(self.my_map, self.starts, self.goals, self.heuristics)
        visualize_map_and_heuristics(self.my_map, self.starts, self.goals, self.heuristics)
        
    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(
            node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint         - use disjoint splitting or not
        """

        self.start_time = timer.time()

        if disjoint:
            splitter = disjoint_splitting
        else:
            splitter = standard_splitting

        print("USING: ", splitter)

        AStar = A_Star

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}

        for i in range(self.num_of_agents):  # Find initial path for each agent
            astar = AStar(self.my_map, self.starts, self.goals, self.heuristics, i, root['constraints'])
            path = astar.find_paths()
            # print(f'i: {i}, path: {path[0]}')
            me_map_pos = []
            for i in path[0]: me_map_pos.append(convert_normal_to_pos_p(i))
            print(me_map_pos)
            if path is None:
                raise BaseException('No solutions')
            
            print("Agent ", i)
            print('PATHS: ', root['paths'])
            root['paths'].append(path[0])

        # Initialize all_paths with the root paths
        all_paths = [root['paths']]
        
        # Visualize initial state
        # astar_step(self.my_map, self.starts, self.goals, self.heuristics, root['paths'], root['constraints'], 0)
        # each_astar_step(self.my_map, self.starts, self.goals, self.heuristics, all_paths, root['constraints'], 0)
        
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            # if self.num_of_generated > 50000:
            #     print('reached maximum number of nodes. Returning...')
            #     return None
            print("Open list: ", self.open_list)
            p = self.pop_node()
            if p['collisions'] == []:
                #*******************************************************************************************************
                mypos = {
                    'agent_0': [],
                    'agent_1': [],
                    'agent_2': [],
                    'agent_3': [],
                }
                self.print_results(p)
                # print(len(p["paths"]))
                # print(f'p_paths: {p["paths"]}')
                for pa in range(len(p["paths"])):
                    # print(pa)
                    for i in p["paths"][pa]:
                        mypos[f"agent_{pa}"].append(convert_normal_to_pos_p(i))
                        # print(pa)
                for i in mypos:
                    print(i, mypos[i])
                # number of nodes generated/expanded for comparing implementations
                return p['paths'], self.num_of_generated, self.num_of_expanded
            collision = p['collisions'].pop(0)
            # constraints = standard_splitting(collision)
            # constraints = disjoint_splitting(collision)
            constraints = splitter(collision)
                #*******************************************************************************************************
            
            ############################################################################################################
            # Visualize final solution
                # max_path_length = max(len(path) for path in p['paths'])
                # for step in range(max_path_length):
                #     astar_step(self.my_map, self.starts, self.goals, self.heuristics, p['paths'], p['constraints'], step)
                #     each_astar_step(self.my_map, self.starts, self.goals, self.heuristics, all_paths, p['constraints'], step)
                # return p['paths'], self.num_of_generated, self.num_of_expanded
            ############################################################################################################

            # collision = p['collisions'].pop(0) 
            # constraints = splitter(collision)


            for constraint in constraints:
                q = {'cost': 0,
                     'constraints': [constraint],
                     'paths': [],
                     'collisions': []
                     }
                for c in p['constraints']:
                    if c not in q['constraints']:
                        q['constraints'].append(c)
                for pa in p['paths']:
                    q['paths'].append(pa)

                ai = constraint['agent']
                astar = AStar(self.my_map, self.starts, self.goals,
                              self.heuristics, ai, q['constraints'])
                path = astar.find_paths()

                if path is not None:
                    q['paths'][ai] = path[0]
                    
                     # Visualize each step of the new solution
                    # max_path_length = max(len(path) for path in q['paths'])
                    # for step in range(max_path_length):
                    #     astar_step(self.my_map, self.starts, self.goals, self.heuristics, q['paths'], q['constraints'], step)
                    #     each_astar_step(self.my_map, self.starts, self.goals, self.heuristics, all_paths, q['constraints'], step)
                        
                    # task 4
                    continue_flag = False
                    if constraint['positive']:
                        vol = paths_violate_constraint(constraint, q['paths'])
                        for v in vol:
                            astar_v = AStar(
                                self.my_map, self.starts, self.goals, self.heuristics, v, q['constraints'])
                            path_v = astar_v.find_paths()
                            if path_v is None:
                                continue_flag = True
                            else:
                                q['paths'][v] = path_v[0]
                        if continue_flag:
                            continue
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)
        return None

    def print_results(self, node):
        my_pos = []
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

        print("Solution:")
        for i in range(len(node['paths'])):
            for j in range(i):
                my_pos.append(convert_normal_to_pos_p(node['paths'][i][0]))
            print("agent", i, ": ", node['paths'][i])

def convert_normal_to_pos(pos):
    return (pos[0]*3 - 16.5, pos[1]*3 - 28.5)

def convert_normal_to_pos_p(pos):
    return (pos[0]*3 - 7.5, pos[1]*3 - 67.5)
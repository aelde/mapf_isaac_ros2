import time as timer
import heapq
from itertools import product
import numpy as np
import copy
import matplotlib.pyplot as plt
import os
from all_planner.DICISION import RESULT_DIR,DIR_COST

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)] #(map cordi) lelf,down,right,up,same
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
        if(len(path)>1):
            assert path[-1] != path[-2]
    return rst

    
def convert_normal_to_pos_w(pos):
    return (pos[0]*3 - 16.5, pos[1]*3 - 28.5)

def convert_normal_to_pos_p(pos):
    return (pos[0]*3 - 7.5, pos[1]*3 - 67.5)

def compute_heuristics(my_map, goal):
    # print(f'GOAL is : {goal} , {convert_normal_to_pos(goal)}')
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    # print
    closed_list[goal] = root
    count = 0
    while len(open_list) > 0:
        count += 1
        # print(f'count: {count}')
        # print(f'o_ {count-1}, {open_list}')
        (cost, loc, curr) = heapq.heappop(open_list)
        # for i,j in enumerate(open_list): print(f'o_ {i}, {j}')
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # print(f'len open is : {len(open_list)}')
    # print(f'len close is : {len(closed_list)}')
    # print(f'close: of {goal} , {convert_normal_to_pos(goal)}')
    # for i,j in enumerate(closed_list.items()): 
        # print(f'c_ {i}, {j}')
    
    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node,meta_agent):
    path = []
    for i in range(len(meta_agent)):
        path.append([])
    curr = goal_node
    while curr is not None:
        for i in range(len(meta_agent)):
            path[i].append(curr['loc'][i])
        curr = curr['parent']
    for i in range(len(meta_agent)):
        path[i].reverse()
        assert path[i] is not None

        print(f'get_path: {path[i]}')

        if len(path[i]) > 1: 
            # remove trailing duplicates
            while path[i][-1] == path[i][-2]:
                path[i].pop()
                print(path[i])
                if len(path[i]) <= 1:
                    break
            # assert path[i][-1] != path[i][-2] # no repeats at the end!!

    assert path is not None
    return path

class A_Star(object):

    def __init__(self,my_map,starts,goals,heuristics,agents,contraints,g_cost=DIR_COST):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations for CBS
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations for CBS
        agents      - the agent (CBS) or meta-agent of the agent (MA-CBS) involved in collision
        constraints - list of dict constraints generated by a CBS splitter; dict = {agent,loc,timestep,positive}
        """            

        self.my_map = my_map


        self.num_generated = 0
        self.num_expanded = 0
        self.CPU_time = 0

        self.open_list = []
        self.closed_list = dict()

        self.g_cost = g_cost
        print(f'G_COST: {g_cost}')
        # print(f'str_g: {g_cost["straight"]}')
        # print(f'le_g: {g_cost["rotate_left"]}')
        # print(f'ri_g: {g_cost["rotate_right"]}')
        
        self.constraints = contraints # to be used to create c_table

        self.agents = agents

        # check if meta_agent is only a simple agent (from basic CBS)
        if not isinstance(agents, list):
            self.agents = [agents]
            # print(meta_agent)

            # add meta_agent keys to constraints
            for c in self.constraints:
                c['meta_agent'] = {c['agent']}

        # FILTER BY INDEX FOR STARTS AND GOALS AND HEURISTICS
        self.starts = [starts[a] for a in self.agents]
        self.heuristics = [heuristics[a] for a in self.agents]
        self.goals = [goals[a] for a in self.agents]

        self.c_table = [] # constraint table
        self.max_constraints = np.zeros((len(self.agents),), dtype=int)


    def push_node(self, node):
        # print('hey push_node...')
        f_value = node['g_val'] + node['h_val']
        print(f'f: {f_value}')
        heapq.heappush(self.open_list, (f_value, node['h_val'], node['loc'], self.num_generated, node))
        self.num_generated += 1
        # print(f'num_gen: {self.num_generated}')
        
    def pop_node(self):
        _,_,_, id, curr = heapq.heappop(self.open_list)

        self.num_expanded += 1
        print(f'num_expan: {self.num_expanded}')
        return curr

    # return a table that constains the list of constraints of all agents for each time step. 
    def build_constraint_table(self, agent):
        # constraint_table = {}
        constraint_table = dict()

        if not self.constraints:
            return constraint_table
        for constraint in self.constraints:
            timestep = constraint['timestep']

            t_constraint = []
            if timestep in constraint_table:
                t_constraint = constraint_table[timestep]

            # positive constraint for agent
            if constraint['positive'] and constraint['agent'] == agent:
                t_constraint.append(constraint)
                constraint_table[timestep] = t_constraint
            # and negative (external) constraint for agent
            elif not constraint['positive'] and constraint['agent'] == agent:
                t_constraint.append(constraint)
                constraint_table[timestep] = t_constraint
            # enforce positive constraints from other agents (i.e. create neg constraint)
            elif constraint['positive']: 
                neg_constraint = copy.deepcopy(constraint)
                neg_constraint['agent'] = agent
                # if edge collision
                if len(constraint['loc']) == 2:
                    # switch traversal direction
                    prev_loc = constraint['loc'][1]
                    curr_loc = constraint['loc'][0]
                    neg_constraint['loc'] = [prev_loc, curr_loc]
                neg_constraint['positive'] = False
                t_constraint.append(neg_constraint)
                constraint_table[timestep] = t_constraint
        
        return constraint_table
                

    # returns if a move at timestep violates a "positive" or a "negative" constraint in c_table
    def constraint_violated(self, curr_loc, next_loc, timestep, c_table_agent, agent):

        # print("the move : {}, {}".format(curr_loc, next_loc))


        if timestep not in c_table_agent:
            return None
        
        for constraint in c_table_agent[timestep]:
            
            if agent == constraint['agent']:
                # vertex constraint
                if len(constraint['loc']) == 1:
                    # positive constraint
                    if constraint['positive'] and next_loc != constraint['loc'][0]:
                        # print("time {} positive constraint : {}".format(timestep, constraint))
                        return constraint
                    # negative constraint
                    elif not constraint['positive'] and next_loc == constraint['loc'][0]:
                        # print("time {} negative constraint : {}".format(timestep, constraint))
                        return constraint
                # edge constraint
                else:
                    if constraint['positive'] and constraint['loc'] != [curr_loc, next_loc]:
                        # print("time {} positive constraint : {}".format(timestep, constraint))
                        return constraint
                    if not constraint['positive'] and constraint['loc'] == [curr_loc, next_loc]:
                        # print("time {} negative constraint : {}".format(timestep, constraint))
                        return constraint

        return None

    # returns whether an agent at goal node at current timestep will violate a constraint in next timesteps
    def future_constraint_violated(self, curr_loc, timestep, max_timestep, c_table_agent, agent):

        for t in range(timestep+1, max_timestep+1):
            if t not in c_table_agent:
                continue

            for constraint in c_table_agent[t]:
        
                if agent == constraint['agent']:
                    # vertex constraint
                    if len(constraint['loc']) == 1:
                        # positive constraint
                        if constraint['positive'] and curr_loc != constraint['loc'][0]:
                            # print("future time {} positive constraint : {}".format(t, constraint))
                            return True
                        # negative constraint
                        elif not constraint['positive'] and curr_loc == constraint['loc'][0]:
                            # print("time {} negative constraint : {}".format(timestep, constraint))
                            # print("future time {} negative constraint : {}".format(t, constraint))
                            return True


        return False

            
    def generate_child_nodes(self, curr):
        
        children = []
        ma_dirs = product(list(range(5)), repeat=len(self.agents)) # directions for move() for each agent: 0, 1, 2, 3, 4
        
        for dirs in ma_dirs: 
            # print(dirs)
            invalid_move = False
            child_loc = []
            # move each agent for new timestep & check for (internal) conflicts with each other
            for i, a in enumerate(self.agents):           
                    aloc = move(curr['loc'][i], dirs[i])
                    # vertex collision; check for duplicates in child_loc
                    if aloc in child_loc:
                        invalid_move = True
                        # print("internal conflict")
                        break
                    child_loc.append(move(curr['loc'][i], dirs[i]))   


            if invalid_move:
                continue


            for i, a in enumerate(self.agents):   
                # edge collision: check for matching locs in curr_loc and child_loc between two agents
                for j, a in enumerate(self.agents):   
                    if i != j:
                        # print(ai, aj)
                        if child_loc[i] == curr['loc'][j] and child_loc[j] == curr['loc'][i]:
                            invalid_move = True             
            
            if invalid_move:
                continue

            # check map constraints and external constraints
            for i, a in enumerate(self.agents):  
                next_loc= child_loc[i]
                # agent out of map bounds
                if next_loc[0]<0 or next_loc[0]>=len(self.my_map) or next_loc[1]<0 or next_loc[1]>=len(self.my_map[0]):
                    invalid_move = True
                # agechild_locnt collison with map obstacle
                elif self.my_map[next_loc[0]][next_loc[1]]:
                    invalid_move = True
                # agent is constrained by a negative external constraint
                elif self.constraint_violated(curr['loc'][i],next_loc,curr['timestep']+1,self.c_table[i], self.agents[i]):
                    invalid_move = True
                if invalid_move:
                    break

            if invalid_move:
                continue

            # find h_values for current moves
            h_value = 0
            for i in range(len(self.agents)):
                    h_value += self.heuristics[i][child_loc[i]]

            h_test = sum([self.heuristics[i][child_loc[i]] for i in range(len(self.agents))])

            assert h_value == h_test

            # g_value = curr['g_val']+ curr['reached_goal'].count(False)
            num_moves = curr['reached_goal'].count(False)
            # print("(edge) cost (curr -> child) in a* tree == ", num_moves)
            
            
            '''*************** new g cost cal ***************'''
            if curr['parent'] is not None:
                parent_locc = np.array(curr['parent']['loc'])
            else:
                parent_locc = np.zeros(2)
                # parent_locc = None  # or some default value
            curr_locc = np.array(curr['loc'][0])
            child_locc = np.array(child_loc[0])
            print(f'family-> parent: {parent_locc}, curr: {curr_locc}, child: {child_locc}')
            
            direc = curr_locc - parent_locc
            new_direction_str = np.array(child_loc) - np.array(curr_locc)
            
            needs_rotation = not np.array_equal(direc, new_direction_str) and not np.array_equal(direc, np.zeros(2))
            cross_product = np.cross(direc, new_direction_str)
            
            # print(f'str_g: {g_cost["straight"]}')
            # print(f'le_g: {g_cost["rotate_left"]}')
            # print(f'ri_g: {g_cost["rotate_right"]}')
            
            stop_cost = self.g_cost["stop"]
            straight_cost = self.g_cost["straight"]
            rotate_left_cost = self.g_cost["rotate_left"]
            rotate_right_cost = self.g_cost["rotate_right"]
            
            custom_g_cost = None
            if dirs == 4:
                print(f'same loc, no move, stop!!! use: {stop_cost}')
                custom_g_cost = stop_cost
            elif cross_product == 0:
                pass
                print(f'parent loc,bACkward??? , no need rot! use: {straight_cost}')
                custom_g_cost = straight_cost
                # continue
            elif needs_rotation:			
                # print(f'cross_product: {cross_product}')
                rotation_cost = rotate_left_cost if cross_product > 0 else rotate_right_cost
                custom_g_cost = rotation_cost
                print(f"need rotation! , NoMove Just rOtatet use: {rotation_cost}")
                child_loc = curr
            else : 
                pass
                print(f"STR, no need rotation! use: {straight_cost}")
                custom_g_cost = straight_cost
                # continue
            
            
            # g_value = curr['g_val'] + num_moves
            g_value = curr['g_val'] + custom_g_cost
            '''***************"***************"***************'''

            print(f'h: {h_value}, g: {g_value}, num_mov:{num_moves}, loc: {child_loc}, {convert_normal_to_pos_p(child_loc[0])}')

            reached_goal = [False for i in range(len(self.agents))]

            for i, a in enumerate(self.agents):
                
                if not reached_goal[i] and child_loc[i] == self.goals[i]:

                    if curr['timestep']+1 <= self.max_constraints[i]:
                        if not self.future_constraint_violated(child_loc[i], curr['timestep']+1, self.max_constraints[i] ,self.c_table[i], self.agents[i]):
                    # print("agent ", a, 'has found solution at timestep ', curr['timestep'] + 1)
                    # print ('MAX CONSTRIANT:', self.max_constraints[i])
                            reached_goal[i] = True
                            # self.max_constraints[i] differs for each node
                    else:
                        reached_goal[i] = True


            child = {'loc': child_loc,
                    'g_val': g_value, # number of new locs (cost) added
                    'h_val': h_value,
                    'parent': curr,
                    'timestep': curr['timestep']+1,
                    'reached_goal': copy.deepcopy(reached_goal)
                    } 

            children.append(child)

            # print(f'children: \n{children}')
        return children

    def compare_nodes(self, n1, n2):
        """Return true is n1 is better than n2."""

        # print(n1['g_val'] + n1['h_val'])
        # print(n2['g_val'] + n2['h_val'])

        assert isinstance(n1['g_val'] + n1['h_val'], int)
        assert isinstance(n2['g_val'] + n2['h_val'], int)

        return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

    def find_paths(self):

        self.start_time = timer.time()

        # print(f'SELF.AGENTS: {self.agents}, {self.starts[0]}, {convert_normal_to_pos(self.starts[0])} ')

        print("> build constraint table")
        

        for i, a in enumerate(self.agents):
            table_i = self.build_constraint_table(a)
            print(table_i)
            self.c_table.append(table_i)
            if table_i.keys():
                self.max_constraints[i] = max(table_i.keys())


        h_value = sum([self.heuristics[i][self.starts[i]] for i in range(len(self.agents))])
        # print(f'h_value: {h_value}')
        
        # assert h_value == h_test

        root = {'loc': [self.starts[j] for j in range(len(self.agents))],
                # 'F_val' : h_value, # only consider children with f_val == F_val
                'g_val': 0, 
                'h_val': h_value, 
                'parent': None,
                'timestep': 0,
                'reached_goal': [False for i in range(len(self.agents))]
                }

        # check if any any agents are already at goal loc
        for i, a in enumerate(self.agents):
            if root['loc'][i] == self.goals[i]:

                if root['timestep'] <= self.max_constraints[i]:
                    if not self.future_constraint_violated(root['loc'][i], root['timestep'], self.max_constraints[i] ,self.c_table[i], self.agents[i]):
                        root['reached_goal'][i] = True

                        self.max_constraints[i] = 0


        self.push_node(root)
        self.closed_list[(tuple(root['loc']),root['timestep'])] = [root]
        # print(f'initial close: {self.closed_list}, len: {len(self.closed_list)}')
        # print(f'initial open: {self.open_list}, len: {len(self.open_list)}')
        
        step_count = 0
        while len(self.open_list) > 0:

            # if num_node_generated >= 30:
            #     return

            curr = self.pop_node()
            print(f'pop: {curr["loc"]}, {convert_normal_to_pos_p(curr["loc"][0])}, g:{curr["g_val"]}, h:{curr["h_val"]}')
            
            solution_found = all(curr['reached_goal'][i] for i in range(len(self.agents)))
            # print(curr['reached_goal'] )

            if solution_found:
                print(f'end... agent: {self.agents}')
                # convert_normal_to_pos()
                print(f'end_cuur ag_{self.agents}: \n{curr}')
                return get_path(curr,self.agents)


            children = self.generate_child_nodes(curr)

            for child in children:

                f_value = child['g_val'] + child['h_val']

                # if (tuple(child['loc']),child['timestep']) in self.closed_list:
                #     existing_node = self.closed_list[(tuple(child['loc']),child['timestep'])]
                #     if self.compare_nodes(child, existing_node):
                #         self.closed_list[(tuple(child['loc']),child['timestep'])] = child
                #         self.push_node(child)
                # else:
                #     # print('bye child ',child['loc'])
                #     self.closed_list[(tuple(child['loc']),child['timestep'])] = child
                #     self.push_node(child)

                if (tuple(child['loc']),child['timestep']) in self.closed_list:
                    existing = self.closed_list[(tuple(child['loc']),child['timestep'])]
                    if (child['g_val'] + child['h_val'] < existing['g_val'] + existing['h_val']) and (child['g_val'] < existing['g_val']) and child['reached_goal'].count(False) <= existing['reached_goal'].count(False):
                        print("child is better than existing in closed list")
                        self.closed_list[(tuple(child['loc']),child['timestep'])] = child
                        self.push_node(child)
                else:
                    # print('bye child ',child['loc'])
                    self.closed_list[(tuple(child['loc']),child['timestep'])] = child
                    self.push_node(child)

                # if (tuple(child['loc']),child['timestep']) not in self.closed_list:
                #     # existing_node = self.closed_list[(tuple(child['loc']),child['timestep'])]
                #     # if compare_nodes(child, existing_node):
                #     self.closed_list[(tuple(child['loc']),child['timestep'])] = child
                #     # print('bye child ',child['loc'])
                #     self.push_node(child)

            # if (tuple(curr['loc']),curr['timestep']) not in self.closed_list:
            #     self.closed_list[(tuple(curr['loc']),curr['timestep'])] = curr
            
            # self.visualize_each_step(curr, self.my_map, self.heuristics[0] ,children, step_count)
            step_count += 1
        print('no solution')

        # print("\nEND OF A*\n") # comment out if needed
        return None        

    def visualize_each_step(self, curr, my_map, h_cost, children, step_count):
        
        # print(f'close_list vi: {self.closed_list}')
        n_x, n_y = len(my_map), len(my_map[0])
        
        fig, ax = plt.subplots(figsize=(n_y, n_x))
        ax.set_title(f'A* Step {step_count} for Robot {self.agents[0]+1}')
        
        # Set up the plot
        ax.set_xticks(np.arange(0, n_y, 1))
        ax.set_yticks(np.arange(0, n_x, 1))
        ax.grid(True)
        
        # Calculate g_cost and f_cost for each explored node in closed_list
        g_cost = {}
        f_cost = {}
        explored_positions = set()
        previous_curr_positions = set()
        for (loc, timestep), nodes in self.closed_list.items():
            if isinstance(nodes, list):
                for node in nodes:
                    for pos in loc:
                        explored_positions.add(pos)
                        if node['parent'] is None:  # This is a previous curr node
                            previous_curr_positions.add(pos)
                        if pos not in g_cost:
                            g_cost[pos] = set()
                            f_cost[pos] = set()
                        g_cost[pos].add(node['g_val'])
                        f_cost[pos].add(node['g_val'] + node['h_val'])
            else:
                node = nodes
                for pos in loc:
                    explored_positions.add(pos)
                    if node['parent'] is None:  # This is a previous curr node
                        previous_curr_positions.add(pos)
                    if pos not in g_cost:
                        g_cost[pos] = set()
                        f_cost[pos] = set()
                    g_cost[pos].add(node['g_val'])
                    f_cost[pos].add(node['g_val'] + node['h_val'])
        
        # Plot heuristic values, g_cost, and f_cost
        for x in range(n_x):
            for y in range(n_y):
                if (x, y) in h_cost:
                    h_value = h_cost[(x, y)]
                    g_values = g_cost.get((x, y), set())
                    f_values = f_cost.get((x, y), set())
                    
                    g_str = ','.join(map(str, sorted(g_values)))
                    f_str = ','.join(map(str, sorted(f_values)))
                    
                    if len(sorted(g_values)) > 0:
                        print(f'x_y: {x},{y} :: g: {g_values}, f: {f_values}')
                        g_str = str(sorted(g_values)[0])
                        f_str = str(sorted(f_values)[0])
                        g_str = str(round(float(g_str),2))
                        f_str = str(round(float(f_str),2))
                    
                    h_value = str(round(float(h_value),2))

                    cell_text = f'h:{h_value}\ng:{g_str}\nf:{f_str}'
                    ax.text(y, x, cell_text, ha='center', va='center', fontsize=13, 
                            color='red' if my_map[x][y] else 'black')

        # Plot obstacles
        obstacle_mask = np.array(my_map, dtype=bool)
        ax.imshow(obstacle_mask, cmap='binary', alpha=0.3)
        
        # Plot explored nodes as small red rectangles
        for pos in explored_positions:
            rect = plt.Rectangle((pos[1] - 0.4, pos[0] - 0.4), 0.8, 0.8, 
                                fill=False, facecolor='green', edgecolor='green', alpha=0.3)
            ax.add_patch(rect)
        
        # Plot start and goal positions
        colors = ['g', 'r', 'b']  # Colors for each agent
        markers = ['o', 's', '^']  # Markers for each agent
        for i, agent in enumerate(self.agents):
            start = self.starts[i]
            goal = self.goals[i]
            ax.plot(start[1], start[0], color=colors[i], marker=markers[i], markersize=10, label=f'Start {agent}')
            ax.plot(goal[1], goal[0], color=colors[i], marker='*', markersize=12, label=f'Goal {agent}')
        
        # Plot current positions and path from start
        for i, agent in enumerate(self.agents):
            pos = curr['loc'][i]
            ax.plot(pos[1], pos[0], color=colors[i], marker=markers[i], markersize=8, fillstyle='none')
            
            # Draw path from start to current position
            path = []
            node = curr
            while node:
                path.append(node['loc'][i])
                node = node['parent']
            path.reverse()
            path_x = [p[1] for p in path]
            path_y = [p[0] for p in path]
            ax.plot(path_x, path_y, color=colors[i], linewidth=2, alpha=0.7)
        
        # Add yellow rectangles for the current path
        for path_pos in path:
            rect = plt.Rectangle((path_pos[1] - 0.4, path_pos[0] - 0.4), 0.8, 0.8, 
                                fill=True, facecolor='green', edgecolor='green', alpha=0.2)
            ax.add_patch(rect)
        
        # Plot children positions and lines to current position
        for child in children:
            for i, agent in enumerate(self.agents):
                pos = child['loc'][i]
                ax.plot(pos[1], pos[0], color=colors[i], marker='.', markersize=6, alpha=0.5)
                ax.plot([curr['loc'][i][1], pos[1]], [curr['loc'][i][0], pos[0]], 
                        color=colors[i], linestyle='--', linewidth=1, alpha=0.5)
        
        ax.legend(loc='upper left', bbox_to_anchor=(1, 1))
        ax.set_aspect('equal', 'box')
        
        # Create result directory if it doesn't exist
        result_dir = RESULT_DIR + f'/step/'
        os.makedirs(result_dir, exist_ok=True)
        
        # Save the plot for each agent
        for agent in self.agents:
            agent_dir = os.path.join(result_dir, f'robot_{agent+1}')
            os.makedirs(agent_dir, exist_ok=True)
            filename = os.path.join(agent_dir, f'step_{step_count:04d}.png')
            plt.savefig(filename, dpi=300, bbox_inches='tight')
        
        plt.close(fig)
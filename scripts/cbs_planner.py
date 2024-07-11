from astar_planner import AstarPlanner
import heapq
import random
import time as timer
from map.map import obstacles_1

def detect_collisions(paths):
        collisions = []
        for i in range(len(paths) - 1):
            for j in range(i + 1, len(paths)):
                collision = detect_collision(paths[i], paths[j])
                if collision:
                    position, t = collision
                    collisions.append({'a1': i, 'a2': j, 'loc': position, 'timestep': t + 1})
        print(f"Collision detected between agents {i} and {j} at {position} at timestep {t + 1}")
        # print(f"Collisions detectedsss: {collisions}")
        return collisions

# def detect_collision(path1, path2):
#     t_range = max(len(path1), len(path2))
#     for t in range(t_range - 1):  # -1 because we're checking t and t+1
#         loc_c1 = get_location(path1, t)
#         loc_c2 = get_location(path2, t)
#         loc1 = get_location(path1, t + 1)
#         loc2 = get_location(path2, t + 1)

#         # vertex collision
#         if loc1 == loc2:
#             return [loc1], t

#         # edge collision
#         if [loc_c1, loc1] == [loc2, loc_c2]:
#             return [loc2, loc_c2], t

#     return None

def detect_collision(path1, path2):
    t_range = max(len(path1), len(path2))
    for t in range(t_range):
        pos1 = get_location(path1, t)
        pos2 = get_location(path2, t)
        
        # Check for vertex collision
        if pos1 == pos2:
            return pos1, t
        
        # Check for edge collision
        if t > 0:
            prev_pos1 = get_location(path1, t-1)
            prev_pos2 = get_location(path2, t-1)
            if pos1 == prev_pos2 and pos2 == prev_pos1:
                return (pos1, pos2), t

    return None

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

# def get_location(path, time):
#     if time < 0:
#         return path[0]
#     elif time < len(path):
#         return path[time]
#     else:
#         return path[-1]  # wait at the goal location

def get_location(path, t):
    if t < len(path):
        return tuple(path[t][:2])  # Assuming path contains [x, y, z] coordinates
    return tuple(path[-1][:2])  # If t is out of range, return the last location

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
        if(len(path)>1):
            assert path[-1] != path[-2]
    return rst

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
    if len(collision['loc'])==1:
        constraints.append({'agent':collision['a1'],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        constraints.append({'agent':collision['a1'],
                            'loc':[collision['loc'][0],collision['loc'][1]],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
        constraints.append({'agent':collision['a2'],
                            'loc':[collision['loc'][1],collision['loc'][0]],
                            'timestep':collision['timestep'],
                            'positive':False
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
    agent = random.randint(0,1)
    a = 'a'+str(agent +1)
    if len(collision['loc'])==1:
        constraints.append({'agent':collision[a],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':True
                            })
        constraints.append({'agent':collision[a],
                            'loc':collision['loc'],
                            'timestep':collision['timestep'],
                            'positive':False
                            })
    else:
        if agent ==0:
            constraints.append({'agent':collision[a],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[a],
                                'loc':[collision['loc'][0],collision['loc'][1]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
        else:
            constraints.append({'agent':collision[a],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':True
                                })
            constraints.append({'agent':collision[a],
                                'loc':[collision['loc'][1],collision['loc'][0]],
                                'timestep':collision['timestep'],
                                'positive':False
                                })
    return constraints

class CBSPlanner:
    def __init__(self):
        self.low_level_planner = AstarPlanner(obstacles=obstacles_1)  # Initialize with empty obstacles

        self.open_list = []
        
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        
    def plan(self, tasks, tb_pos):
        # Implement CBS algorithm here
        # Use self.low_level_planner for low-level search
        pass

    def resolve_conflicts(self, paths):
        # Implement conflict resolution logic
        pass

    def update_constraints(self, constraints):
        # Update constraints for low-level search
        pass
    
    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node


    def find_solution(self, disjoint, Paths, Collisions):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint         - use disjoint splitting or not
        """

        self.start_time = timer.time()
        
        if disjoint:
            splitter = disjoint_splitting
        else:
            splitter = standard_splitting

        print("USING: ", splitter)

        # AStar = A_Star

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}

        root['paths'] = Paths
        root['collisions'] = Collisions
        
        # for i in range(self.num_of_agents):  # Find initial path for each agent
        #     astar = AStar(self.my_map, self.starts, self.goals, self.heuristics,i, root['constraints'])
        # astar = AstarPlanner(obstacles=obstacles_1)
        #     path = astar.find_paths()

            # if path is None:
            #     raise BaseException('No solutions')
            # root['paths'].append(path[0])

        root['cost'] = get_sum_of_cost(root['paths'])
        # root['collisions'] = detect_collisions(root['paths'])
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
            p = self.pop_node()
            if p['collisions'] == []:
                self.print_results(p)
                for pa in p['paths']:
                    print(pa)
                return p['paths'], self.num_of_generated, self.num_of_expanded # number of nodes generated/expanded for comparing implementations
            collision = p['collisions'].pop(0)
            # constraints = standard_splitting(collision)
            # constraints = disjoint_splitting(collision)
            constraints = splitter(collision)

            for constraint in constraints:
                q = {'cost':0,
                    'constraints': [constraint],
                    'paths':[],
                    'collisions':[]
                }
                for c in p['constraints']:
                    if c not in q['constraints']:
                        q['constraints'].append(c)
                for pa in p['paths']:
                    q['paths'].append(pa)
                
                ai = constraint['agent']
                # astar = AStar(self.my_map,self.starts, self.goals,self.heuristics,ai,q['constraints'])
                # path = astar.find_paths()
                astar = AstarPlanner(obstacles=obstacles_1)
                path = astar.plan()

                if path is not None:
                    q['paths'][ai]= path[0]
                    # task 4
                    continue_flag = False
                    if constraint['positive']:
                        vol = paths_violate_constraint(constraint,q['paths'])
                        for v in vol:
                            # astar_v = AStar(self.my_map,self.starts, self.goals,self.heuristics,v,q['constraints'])
                            # path_v = astar_v.find_paths()
                            astar = AstarPlanner(obstacles=obstacles_1)
                            path_v = astar.find_paths()
                            if path_v  is None:
                                continue_flag =True
                            else:
                                q['paths'][v] = path_v[0]
                        if continue_flag:
                            continue
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)     
        return None

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

        print("Solution:")
        for i in range(len(node['paths'])):
            print("agent", i, ": ", node['paths'][i])

    # Add other necessary helper methods
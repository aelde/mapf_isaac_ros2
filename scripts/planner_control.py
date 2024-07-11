from astar_planner import AstarPlanner
from cbs_planner import CBSPlanner, disjoint_splitting, standard_splitting, detect_collision, detect_collisions
import random

class PlannerControl:
    def __init__(self, obstacles):
        self.astar_planner = AstarPlanner(obstacles)
        self.cbs_planner = CBSPlanner()
        self.obstacles = obstacles

    def plan_path(self, task, tb_pos):
        tb_id, goal_pos = task
        start_pos = next((tb["curr_pos"] for tb in tb_pos if tb["id"] == tb_id), None)
        
        if start_pos is None:
            print(f"Error: No position found for tb_id {tb_id}")
            return None, None

        path, _, _ = self.astar_planner.plan((tb_id, goal_pos), tb_pos)
        return path, start_pos


        return all_paths

    # def plan_multiple_paths(self, tasks, tb_pos):
    #     root = {'cost': 0,
    #             'constraints': [],
    #             'paths': [],
    #             'collisions': []}
    #     # all_paths = []
    #     # is_collision = []
    #     for task in tasks:
    #         path, _ = self.plan_path(task, tb_pos)
    #         print(f"task: {task}")
    #         # print(f'start_pos: {_}')
    #         # print(f"Path: {path}")
            
    #         if path:
    #             root['paths'].append(path)
    #             # all_paths.append(path)
    #         else:
    #             print(f"Failed to find path for task: {task}")

    #     # Log A* planning results
    #     # self.astar_planner.log_overall_planning()

    #     root['collisions'] = detect_collisions(root['paths'])
    #     if root['collisions']:
    #         print("Collisions detected. Using CBS planner.")
    #         cbs_paths = self.cbs_planner.find_solution(True, root['paths'], root['collisions'])
    
    def plan_multiple_paths(self, tasks, tb_pos):
        all_paths = []
        for task in tasks:
            path, _ = self.plan_path(task, tb_pos)
            print(f"task: {task}")
            if path:
                all_paths.append(path)
            else:
                print(f"Failed to find path for task: {task}")

        collisions = detect_collisions(all_paths)
        print(f"Collisions: {collisions}")
        if collisions:
            print("Collisions detected. Using CBS planner.")
            cbs_paths = self.cbs_planner.find_solution(True, all_paths, collisions)
            return cbs_paths
        else:
            return all_paths
    
        # Log the overall planning results
        # self.log_overall_planning(tasks, all_paths, collisions)

    def get_location(self, path, t):
        if t < len(path):
            return tuple(path[t][:2])  # Assuming path contains [x, y, z] coordinates
        return tuple(path[-1][:2])  # If t is out of range, return the last location

    # def log_overall_planning(self, tasks, paths, collisions):
    #     print("Overall Planning Results:")
    #     print(f"Number of tasks: {len(tasks)}")
    #     print(f"Number of paths found: {len(paths)}")
    #     print(f"Number of collisions detected: {len(collisions)}")
        
    #     for i, (task, path) in enumerate(zip(tasks, paths)):
    #         print(f"Task {i + 1}:")
    #         print(f"  Start: {task[1]}")
    #         print(f"  Goal: {path[-1]}")
    #         print(f"  Path length: {len(path)}")
        
    #     if collisions:
    #         print("Collisions:")
    #         for collision in collisions:
    #             print(f"  Agents {collision['a1']} and {collision['a2']} at {collision['loc']} at timestep {collision['timestep']}")
        
        # You might also want to write this information to a file
        # with open('planning_log.txt', 'a') as f:
        #     f.write("... log content ...")

    # Add other necessary methods
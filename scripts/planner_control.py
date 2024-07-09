from astar_planner import AstarPlanner
from cbs_planner import CBSPlanner

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

    def plan_multiple_paths(self, tasks, tb_pos):
        all_paths = []
        for task in tasks:
            path, _ = self.plan_path(task, tb_pos)
            if path:
                all_paths.append(path)
            else:
                print(f"Failed to find path for task: {task}")

        # Log A* planning results
        self.astar_planner.log_overall_planning()

        collisions = self.detect_collisions(all_paths)
        if collisions:
            print("Collisions detected. Using CBS planner.")
            all_paths = self.cbs_planner.plan(tasks, tb_pos)

        # Log the overall planning results
        self.log_overall_planning(tasks, all_paths, collisions)

        return all_paths

    def detect_collisions(self, paths):
        collisions = []
        for i in range(len(paths) - 1):
            for j in range(i + 1, len(paths)):
                collision = self.detect_collision(paths[i], paths[j])
                if collision:
                    position, t = collision
                    collisions.append({'a1': i, 'a2': j, 'loc': position, 'timestep': t + 1})
        print(f"Collisions detectedsss: {collisions}")
        return collisions

    def detect_collision(self, path1, path2):
        t_range = max(len(path1), len(path2))
        for t in range(t_range - 1):  # -1 because we're checking t and t+1
            loc_c1 = self.get_location(path1, t)
            loc_c2 = self.get_location(path2, t)
            loc1 = self.get_location(path1, t + 1)
            loc2 = self.get_location(path2, t + 1)

            # vertex collision
            if loc1 == loc2:
                return [loc1], t

            # edge collision
            if [loc_c1, loc1] == [loc2, loc_c2]:
                return [loc2, loc_c2], t

        return None

    def get_location(self, path, t):
        if t < len(path):
            return tuple(path[t][:2])  # Assuming path contains [x, y, z] coordinates
        return tuple(path[-1][:2])  # If t is out of range, return the last location

    def log_overall_planning(self, tasks, paths, collisions):
        print("Overall Planning Results:")
        print(f"Number of tasks: {len(tasks)}")
        print(f"Number of paths found: {len(paths)}")
        print(f"Number of collisions detected: {len(collisions)}")
        
        for i, (task, path) in enumerate(zip(tasks, paths)):
            print(f"Task {i + 1}:")
            print(f"  Start: {task[1]}")
            print(f"  Goal: {path[-1]}")
            print(f"  Path length: {len(path)}")
        
        if collisions:
            print("Collisions:")
            for collision in collisions:
                print(f"  Agents {collision['a1']} and {collision['a2']} at {collision['loc']} at timestep {collision['timestep']}")
        
        # You might also want to write this information to a file
        # with open('planning_log.txt', 'a') as f:
        #     f.write("... log content ...")

    # Add other necessary methods
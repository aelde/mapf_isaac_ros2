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
        
        print(f'Start planning tb: {tb_id} path...')
        
        if start_pos is None:
            print(f"Error: No position found for tb_id {tb_id}")
            return None, None

        path, _, _ = self.astar_planner.plan((tb_id, goal_pos), tb_pos)
        
        if path:
            if not self.detect_collisions([path]):
                print(f"Path found for tb_id {tb_id} ...")
                return path, start_pos
            else:
                print("Collisions detected. Using CBS planner.")
                all_paths = self.cbs_planner.plan([task], tb_pos)
                return all_paths[0] if all_paths else None, start_pos
        else:
            return None, start_pos

    def plan_multiple_paths(self, tasks, tb_pos):
        all_paths = []
        for task in tasks:
            path, _ = self.plan_path(task, tb_pos)
            if path:
                all_paths.append(path)

        collisions = self.detect_collisions(all_paths)
        print(f"Collisions: {collisions}")
        if collisions:
            print("Collisions detected. Using CBS planner.")
            all_paths = self.cbs_planner.plan(tasks, tb_pos)

        return all_paths

    def detect_collisions(self, paths): # this function is used to detect collisions all paths
        collisions = []
        for i in range(len(paths) - 1):
            for j in range(i + 1, len(paths)):
                collision = self.detect_collision(paths[i], paths[j])
                if collision:
                    position, t = collision
                    collisions.append({'a1': i, 'a2': j, 'loc': position, 'timestep': t + 1})
        return collisions

    def detect_collision(self, path1, path2): # this function is used to detect collision between two paths
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

    def log_overall_planning(self):
        # Implement logging logic here
        # This method can be called from the ROS node to log planning results
        pass

    # Add other necessary methods
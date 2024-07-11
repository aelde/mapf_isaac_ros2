import numpy as np
import heapq
import time
import os
import yaml
from datetime import datetime

def load_uneven_astar_config():
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'uneven_astar.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config['costs']

class AstarPlanner:
    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.costs = load_uneven_astar_config()
        self.planning_stats = {}

    def log_path_planning(self, tb_id, planning_time, path_length, start_pos, goal_pos, thinking_steps, success=True):
        log_file = f"result/pathplanninglog_cos_{self.costs}.txt"
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(log_file, "a") as f:
            f.write(f"Timestamp: {timestamp}\n")
            f.write(f"Robot ID: TB_{tb_id}\n")
            f.write(f"Planning Time: {planning_time:.4f} sec\n")
            f.write(f"Start Position: {start_pos}\n")
            f.write(f"Goal Position: {goal_pos}\n")
            f.write(f"A* Thinking Steps: {thinking_steps}\n")
            if success:
                f.write(f"Path Length: {path_length} steps\n")
                f.write(f"Straight: {self.costs['straight']}, "
                        f"Rotate Left: {self.costs['rotate_left']}, "
                        f"Rotate Right: {self.costs['rotate_right']}\n")
                f.write(f"Status: Path found successfully\n")
            else:
                f.write("Status: No path found\n")
            f.write(f"{'='*50}\n\n")

    def log_overall_planning(self):
        log_file = f"result/pathplanninglog_cos_{self.costs}.txt"
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        total_planning_time = sum(stats["planning_time"] for stats in self.planning_stats.values())
        total_path_length = sum(stats["path_length"] for stats in self.planning_stats.values())
        total_thinking_steps = sum(stats["thinking_steps"] for stats in self.planning_stats.values())
        successful_plans = sum(1 for stats in self.planning_stats.values() if stats["success"])
        
        with open(log_file, "a") as f:
            f.write(f"{'#'*50}\n")
            f.write(f"OVERALL PLANNING SUMMARY\n")
            f.write(f"{'#'*50}\n")
            f.write(f"Total Robots: {len(self.planning_stats)}\n")
            f.write(f"Successful Plans: {successful_plans}\n")
            f.write(f"Total Planning Time: {total_planning_time:.4f} sec\n")
            f.write(f"Average Planning Time: {total_planning_time/len(self.planning_stats):.4f} sec\n")
            f.write(f"Total Path Length: {total_path_length} steps\n")
            f.write(f"Total A* Thinking Steps: {total_thinking_steps}\n")
            f.write(f"Average A* Thinking Steps: {total_thinking_steps/len(self.planning_stats):.2f}\n")
            f.write(f"{'='*50}\n\n")
        self.planning_stats.clear()

    def is_valid_move(self, pos):
        pos_array = np.array(pos)
        return not np.any(np.all(np.abs(self.obstacles - pos_array) < 1e-6, axis=1))

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def plan(self, tb_job, tb_pos):
        tb_id, goal_pos = tb_job
        start_time = time.time()
        
        curr_tb_node_pos = next((tb["curr_pos"] for tb in tb_pos if tb["id"] == tb_id), None)
        if curr_tb_node_pos is None:
            print(f"Error: No position found for tb_id {tb_id}")
            return [], None, []

        start_pos = tuple(curr_tb_node_pos)
        goal_pos = tuple(goal_pos)
        
        # print(f'Start planning tb: {tb_id} path...')
        # print(f'Current position: {start_pos}')
        # print(f'Goal position: {goal_pos}')

        open_set = []
        heapq.heappush(open_set, (0, self.heuristic(start_pos, goal_pos), start_pos))
        came_from = {}
        g_score = {start_pos: 0}
        f_score = {start_pos: self.heuristic(start_pos, goal_pos)}
        closed_set = set()

        dis_per_grid = 3
        straight_cost = self.costs['straight']
        rotate_left_cost = self.costs['rotate_left']
        rotate_right_cost = self.costs['rotate_right']

        step = 0
        path_steps = []

        while open_set:
            current_f, current_h, current = heapq.heappop(open_set)
            
            path_steps.append((current, open_set.copy(), closed_set.copy(), g_score.copy(), f_score.copy(), came_from.copy()))
            
            step += 1

            if np.linalg.norm(np.array(current)[:2] - np.array(goal_pos)[:2]) < 1e-6:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_pos)
                
                end_time = time.time()
                planning_time = end_time - start_time
                thinking_steps = len(path_steps)-1  # Number of A* thinking steps
                
                # Log path planning result
                # if path:
                #     self.log_path_planning(tb_id, planning_time, len(path)-1, start_pos, goal_pos, thinking_steps, success=True)
                # else:
                #     self.log_path_planning(tb_id, planning_time, 0, start_pos, goal_pos, thinking_steps, success=False)

                # Store planning stats for overall summary
                self.planning_stats[tb_id] = {
                    "planning_time": planning_time,
                    "path_length": len(path) - 1 if path else 0,
                    "thinking_steps": thinking_steps,
                    "success": bool(path)
                }
                
                # return path, start_pos, path_steps
                return path[::-1], start_pos, path_steps

            closed_set.add(current)

            neighbors = [
                tuple(np.array(current) + np.array([dis_per_grid, 0, 0])),
                tuple(np.array(current) - np.array([dis_per_grid, 0, 0])),
                tuple(np.array(current) + np.array([0, dis_per_grid, 0])),
                tuple(np.array(current) - np.array([0, dis_per_grid, 0])),
            ]

            prev_node = came_from.get(current)
            if prev_node:
                direction = np.array(current) - np.array(prev_node)
            else:
                direction = np.zeros(3)

            for neighbor in neighbors:
                if not self.is_valid_move(np.array(neighbor)) or neighbor in closed_set:
                    continue

                new_direction = np.array(neighbor) - np.array(current)
                needs_rotation = not np.array_equal(direction, new_direction) and not np.array_equal(direction, np.zeros(3))

                if needs_rotation:
                    cross_product = np.cross(direction[:2], new_direction[:2])
                    rotation_cost = rotate_left_cost if cross_product > 0 else rotate_right_cost
                    tentative_g_score = g_score[current] + rotation_cost
                    # tentative_g_score = g_score[current] + straight_cost + rotation_cost
                else:
                    tentative_g_score = g_score[current] + straight_cost 

                if neighbor not in [item[2] for item in open_set] or tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    h_score = self.heuristic(neighbor, goal_pos)
                    f_score[neighbor] = g_score[neighbor] + h_score
                    if neighbor not in [item[2] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], h_score, neighbor))
                    else:
                        for i, (f, h, n) in enumerate(open_set):
                            if n == neighbor:
                                open_set[i] = (f_score[neighbor], h_score, neighbor)
                                heapq.heapify(open_set)
                                break

        print("Warning: No path found")
        
        return [], start_pos, path_steps
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import numpy as np
from mapf_isaac.msg import TbPathtoGoal, TbTask, Tbpose2D
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime
from matplotlib.transforms import Affine2D
from matplotlib.patches import Rectangle
import heapq

class AstarPathPlanner(Node):
    def __init__(self):
        super().__init__('Astar_Path_Planner')
        self.sub_task = self.create_subscription(TbTask, 'all_tb_job_task', self.sub_tb_job_callback, 10)
        self.pub_path_goal = self.create_publisher(TbPathtoGoal, 'TbPathtoGoal_top', 10)
        self.sub_tb_pos = self.create_subscription(Tbpose2D, 'tb_pose2d', self.sub_tbs_pos_cb, 10)
        
        self.tb_queue_job_task = []
        self.tb_pos = [
            {"id": i, "name": f"tb_{i}", "curr_pos": np.zeros(3), "initial": np.zeros(3)}
            for i in range(1, 4)
        ]

        # Add obstacle positions
        self.obstacles = np.array([
            [-10.5, 4.5, 0], [-10.5, 7.5, 0], [-10.5, 10.5, 0], [-10.5, 13.5, 0],
            [-10.5, 16.5, 0], [-10.5, 19.5, 0], [-10.5, 22.5, 0],
            [-4.5, 4.5, 0], [-4.5, 7.5, 0], [-4.5, 10.5, 0], [-4.5, 13.5, 0],
            [-4.5, 16.5, 0], [-4.5, 19.5, 0], [-4.5, 22.5, 0],
            [-1.5, 4.5, 0], [-1.5, 7.5, 0], [-1.5, 10.5, 0], [-1.5, 13.5, 0],
            [-1.5, 16.5, 0], [-1.5, 19.5, 0], [-1.5, 22.5, 0],
            [4.5, 4.5, 0], [4.5, 7.5, 0], [4.5, 10.5, 0], [4.5, 13.5, 0],
            [4.5, 16.5, 0], [4.5, 19.5, 0], [4.5, 22.5, 0],
            [7.5, 4.5, 0], [7.5, 7.5, 0], [7.5, 10.5, 0],
            [10.5, 4.5, 0], [10.5, 7.5, 0], [10.5, 10.5, 0], [10.5, 13.5, 0],
            [10.5, 16.5, 0], [10.5, 19.5, 0], [10.5, 22.5, 0],
            [13.5, 4.5, 0], [13.5, 7.5, 0], [13.5, 10.5, 0], [13.5, 13.5, 0],
            [13.5, 16.5, 0], [13.5, 19.5, 0], [13.5, 22.5, 0]
        ])
        
    def visualize_astar_path(self, tb_id, start_pos, goal_pos, current, open_set, closed_set, g_score, f_score, came_from, step):
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_title(f'A* Path Planning for TB_{tb_id} - Step {step}')
        
        # Set up the grid
        ax.set_xlim(-28.5, 28.5)
        ax.set_ylim(-16.5, 16.5)
        ax.set_xticks(np.arange(-25.5, 26.5, 3))
        ax.set_yticks(np.arange(-13.5, 14.5, 3))
        ax.grid(True)

        # Plot obstacles
        obstacles = self.obstacles[:, :2]
        ax.scatter(obstacles[:, 1], obstacles[:, 0], c='k', marker='s', s=100, label='Obstacles')
        
        # Plot start and goal
        ax.scatter(start_pos[1], start_pos[0], c='c', marker='o', s=200, label='Start')
        ax.scatter(goal_pos[1], goal_pos[0], c='m', marker='*', s=200, label='Goal')
        
        # Plot open and closed sets
        open_set_coords = np.array([pos for _, pos in open_set])
        closed_set_coords = np.array(list(closed_set))
        if open_set_coords.size > 0:
            ax.scatter(open_set_coords[:, 1], open_set_coords[:, 0], c='g', marker='s', s=100, alpha=0.3, label='Open Set')
        if closed_set_coords.size > 0:
            ax.scatter(closed_set_coords[:, 1], closed_set_coords[:, 0], c='r', marker='s', s=100, alpha=0.3, label='Closed Set')
        
        # Plot current position
        ax.scatter(current[1], current[0], c='y', marker='o', s=150, label='Current')
        
        # Plot f, g, h values
        for pos in set(list(g_score.keys()) + list(f_score.keys())):
            x, y = pos[1], pos[0]
            g = g_score.get(pos, float('inf'))
            f = f_score.get(pos, float('inf'))
            h = f - g
            ax.add_patch(Rectangle((x-1.5, y-1.5), 3, 3, fill=False, edgecolor='gray'))
            ax.text(x, y+1.3, f'f={f:.1f}', ha='center', va='bottom', fontsize=8)
            ax.text(x, y+0.2, f'g={g:.1f}', ha='center', va='top', fontsize=8)
            ax.text(x, y-0.9, f'h={h:.1f}', ha='center', va='center', fontsize=8)
        
        # Plot path so far
        path = []
        current_pos = current
        while current_pos in came_from:
            path.append(current_pos)
            current_pos = came_from[current_pos]
        path.append(start_pos)
        path = path[::-1]
        path_array = np.array(path)
        ax.plot(path_array[:, 1], path_array[:, 0], 'b-', linewidth=2, label='Path')
        
        ax.set_aspect('equal', 'box')
        ax.set_ylabel('X')
        ax.set_xlabel('Y')
        ax.legend(loc='lower right')
        
        # Add step counter in top left corner
        ax.text(-27.5, 15.5, f'Step: {step}', fontsize=12, fontweight='bold', 
                bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))
        
        # Rotate the plot 90 degrees clockwise
        plt.gca().invert_yaxis()
        
        # Create 'astar_steps' directory if it doesn't exist
        if not os.path.exists('astar_steps'):
            os.makedirs('astar_steps')
        
        # Save the figure
        filename = f'astar_steps/TB_{tb_id}_step_{step:03d}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        print(f'A* step visualization saved as {filename}')

    def visualize_path(self, tb_id, start_pos, goal_pos, path):
        fig, ax = plt.subplots(figsize=(16, 12))  # Adjusted figure size for better visibility
        ax.set_title(f'Path Planning for TB_{tb_id}')
        
        # Convert tuples to numpy arrays for plotting
        start_pos = np.array(start_pos)
        goal_pos = np.array(goal_pos)
        path = np.array(path)
        
        # Set up the grid
        ax.set_xlim(-28.5, 28.5)  # Swapped for rotation
        ax.set_ylim(-16.5, 16.5)  # Swapped for rotation
        ax.set_xticks(np.arange(-25.5, 26.5, 3))
        ax.set_yticks(np.arange(-13.5, 14.5, 3))
        ax.grid(True)

        # Plot obstacles
        obstacles = self.obstacles[:, :2]  # Only use x and y coordinates
        ax.scatter(obstacles[:, 1], obstacles[:, 0], c='k', marker='s', s=100, label='Obstacles')
        
        # Plot start and goal
        ax.scatter(start_pos[1], start_pos[0], c='c', marker='o', s=200, label='Start')
        ax.scatter(goal_pos[1], goal_pos[0], c='m', marker='*', s=200, label='Goal')
        
        # Plot path
        path = np.array(path)
        ax.plot(path[:, 1], path[:, 0], 'y-', linewidth=2, label='Path')
        
        ax.set_aspect('equal', 'box')
        ax.set_ylabel('X')
        ax.set_xlabel('Y')
        ax.legend()
        
        # Rotate the plot 90 degrees clockwise
        plt.gca().invert_yaxis()
        
        # Create 'path' directory if it doesn't exist
        if not os.path.exists('path'):
            os.makedirs('path')
        
        # Generate unique filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'path/TB_{tb_id}_path_{timestamp}.png'
        
        # Save the figure
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)  # Close the figure to free up memory
        
        print(f'Path visualization saved as {filename}')
    
    def visualize_all_paths(self, all_paths, all_starts, all_goals):
        fig, ax = plt.subplots(figsize=(16, 12))  # Swapped dimensions for rotation
        ax.set_title('Path Planning for All TBs')
        
        # Plot paths, starts and goals for each robot
        for i, (path, start, goal) in enumerate(zip(all_paths, all_starts, all_goals)):
            path = np.array(path)
            start = np.array(start)
            goal = np.array(goal)
        
        # Set up the grid
        ax.set_xlim(-28.5, 28.5)  # Swapped for rotation
        ax.set_ylim(-16.5, 16.5)  # Swapped for rotation
        ax.set_xticks(np.arange(-25.5, 26.5, 3))
        ax.set_yticks(np.arange(-13.5, 14.5, 3))
        ax.grid(True)

        # Plot obstacles
        obstacles = self.obstacles[:, :2]  # Only use x and y coordinates
        ax.scatter(obstacles[:, 1], obstacles[:, 0], c='k', marker='s', s=100, label='Obstacles')
        
        # Define color mapping for robot IDs
        # color_map = {0: 'g', 1: 'r', 2: 'b'}
        colors = ['g', 'r', 'b']  # Changed to single character color codes

        # https://matplotlib.org/stable/users/explain/colors/colors.html
        
        
        # Plot paths, starts and goals for each robot
        for i, (path, start, goal) in enumerate(zip(all_paths, all_starts, all_goals)):
            print(f'tb number: {i+1} :: color: {colors[i]} :: start: {start}')
            path = np.array(path)
            if path.size > 0:  # Check if path is not empty
                ax.plot(path[:, 1], path[:, 0], color=colors[i], linestyle='-', linewidth=2, label=f'Path TB_{i+1}')
            ax.scatter(start[1], start[0], c='c', marker='o', s=200)
            ax.scatter(goal[1], goal[0], c='m', marker='*', s=200)
        
        ax.set_aspect('equal', 'box')
        ax.set_xlabel('Y')  # Swapped for rotation
        ax.set_ylabel('X')  # Swapped for rotation
        ax.legend()
        
        # Rotate the plot 90 degrees clockwise
        plt.gca().invert_yaxis()
        
        # Create 'path' directory if it doesn't exist
        if not os.path.exists('path'):
            os.makedirs('path')
        
        # Generate unique filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'path/All_TBs_paths_{timestamp}.png'
        
        # Save the figure
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)  # Close the figure to free up memory
        
        # print(f'All paths visualization saved as {filename}')
    
    def sub_tbs_pos_cb(self, msg):
        for tb in self.tb_pos:
            if tb["id"] == msg.tb_id:
                tb["curr_pos"] = np.array([msg.tb_pos.x, msg.tb_pos.y, 0.0])
                break

    def pub_tb_path(self, tb_ids, paths):
        # for tb_id, path in zip(tb_ids, paths):
        #     msg = Float32MultiArray()
        #     msg_path = TbPathtoGoal()
        #     msg.data = [coord for point in path for coord in point]
        
        for tb_id, path in zip(tb_ids, paths):
            msg = Float32MultiArray()
            msg_path = TbPathtoGoal()
            msg.data = [coord for point in path for coord in point]
            
            dim0 = MultiArrayDimension(label="points", size=len(path), stride=len(msg.data))
            dim1 = MultiArrayDimension(label="coordinates", size=3, stride=3)
            msg.layout.dim = [dim0, dim1]

            msg_path.tb_id = tb_id
            msg_path.listofpath = msg
            self.pub_path_goal.publish(msg_path)

            # print(f'Published path for tb_id: {tb_id}')
            # for point in path:
            #     print(f'{point}')

    def sub_tb_job_callback(self, msg):
        # print(f'Received: start_routing={msg.start_routing}, tb_id={msg.tb_id}, tb_goal={msg.tb_goal}')
        self.tb_queue_job_task.append([msg.tb_id, np.array([msg.tb_goal.x, msg.tb_goal.y, msg.tb_goal.z])])
        # print(f'All job tasks:')
        for task in self.tb_queue_job_task:
            print(task)
        print(f'-----')
        
        if msg.start_routing:
            self.plan_and_publish_paths()

    def plan_and_publish_paths(self):
        all_paths = []
        all_tb_id = []
        all_starts = []
        all_goals = []
        
        for tb_job in self.tb_queue_job_task:
            path, start_pos = self.astar_planner(tb_job)
            if path:
                all_paths.append(path)
                all_tb_id.append(tb_job[0])
                all_starts.append(start_pos)
                all_goals.append(tb_job[1])
                
                # Visualize individual path
                self.visualize_path(tb_job[0], start_pos, tb_job[1], path)
            else:
                print(f"Failed to find path for tb_id: {tb_job[0]}")
        
        # Visualize all paths together
        if all_paths:
            print(f'All paths: ')
            for i in range(len(all_paths)):
                print(f'TB_{all_tb_id[i]}: {all_paths[i]}')
            
            # Sort the paths by tb_id
            sorted_indices = sorted(range(len(all_tb_id)), key=lambda k: all_tb_id[k])
            sorted_paths = [all_paths[i] for i in sorted_indices]
            sorted_starts = [all_starts[i] for i in sorted_indices]
            sorted_goals = [all_goals[i] for i in sorted_indices]
            sorted_tb_id = [all_tb_id[i] for i in sorted_indices]
            
            print(f'/*/*/*/*/*/*/*/*/*/*/*/')
            print(f'Sorted TB IDs: ')
            for i, tb_id in enumerate(sorted_tb_id):
                print(f'TB_{tb_id}: {sorted_paths[i]}')
            
            self.visualize_all_paths(sorted_paths, sorted_starts, sorted_goals)
            self.pub_tb_path(sorted_tb_id, sorted_paths)
        else:
            print("No valid paths found for any robots")
        
        self.tb_queue_job_task = []

    def is_valid_move(self, pos):
        pos_array = np.array(pos)
        return not np.any(np.all(np.abs(self.obstacles - pos_array) < 1e-6, axis=1))
    
    def astar_planner(self, tb_job):
        tb_id, goal_pos = tb_job
        curr_tb_node_pos = next((tb["curr_pos"] for tb in self.tb_pos if tb["id"] == tb_id), None)
        if curr_tb_node_pos is None:
            print(f"Error: No position found for tb_id {tb_id}")
            return [], None

        start_pos = tuple(curr_tb_node_pos)
        goal_pos = tuple(goal_pos)
        
        print(f'Start planning tb: {tb_id} path...')
        print(f'Current position: {start_pos}')
        print(f'Goal position: {goal_pos}')

        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        open_set = []
        heapq.heappush(open_set, (0, start_pos))
        came_from = {}
        g_score = {start_pos: 0}
        f_score = {start_pos: heuristic(start_pos, goal_pos)}
        closed_set = set()

        dis_per_grid = 3
        step = 0

        while open_set:
            current = heapq.heappop(open_set)[1]
            
            self.visualize_astar_path(tb_id, start_pos, goal_pos, current, open_set, closed_set, g_score, f_score, came_from, step)
            step += 1

            if np.linalg.norm(np.array(current)[:2] - np.array(goal_pos)[:2]) < 1e-6:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_pos)
                return path[::-1], start_pos

            closed_set.add(current)

            neighbors = [
                tuple(np.array(current) + np.array([dis_per_grid, 0, 0])),
                tuple(np.array(current) - np.array([dis_per_grid, 0, 0])),
                tuple(np.array(current) + np.array([0, dis_per_grid, 0])),
                tuple(np.array(current) - np.array([0, dis_per_grid, 0])),
            ]

            for neighbor in neighbors:
                if not self.is_valid_move(np.array(neighbor)) or neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + dis_per_grid

                if neighbor not in [item[1] for item in open_set] or tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal_pos)
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print("Warning: No path found")
        return [], start_pos
    
def main(args=None):
    rclpy.init(args=args)
    path_planner = AstarPathPlanner()
    print('A* Path Planner node running')
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        print('A* Path Planner node terminated')
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
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

    def visualize_path(self, tb_id, start_pos, goal_pos, path):
        fig, ax = plt.subplots(figsize=(16, 12))  # Adjusted figure size for better visibility
        ax.set_title(f'Path Planning for TB_{tb_id}')
        
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
        
        # Set up the grid
        ax.set_xlim(-28.5, 28.5)  # Swapped for rotation
        ax.set_ylim(-16.5, 16.5)  # Swapped for rotation
        ax.set_xticks(np.arange(-25.5, 26.5, 3))
        ax.set_yticks(np.arange(-13.5, 14.5, 3))
        ax.grid(True)

        # Plot obstacles
        obstacles = self.obstacles[:, :2]  # Only use x and y coordinates
        ax.scatter(obstacles[:, 1], obstacles[:, 0], c='k', marker='s', s=100, label='Obstacles')
        
        # Colors for different robots
        colors = ['g', 'r', 'b']  # Changed to single character color codes
        # https://matplotlib.org/stable/users/explain/colors/colors.html
        
        
        # Plot paths, starts and goals for each robot
        for i, (path, start, goal) in enumerate(zip(all_paths, all_starts, all_goals)):
            path = np.array(path)
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
        
        print(f'All paths visualization saved as {filename}')
    
    def sub_tbs_pos_cb(self, msg):
        for tb in self.tb_pos:
            if tb["id"] == msg.tb_id:
                tb["curr_pos"] = np.array([msg.tb_pos.x, msg.tb_pos.y, 0.0])
                break

    def pub_tb_path(self, tb_ids, paths):
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

            print(f'Published path for tb_id: {tb_id}')
            for point in path:
                print(f'{point}')

    def sub_tb_job_callback(self, msg):
        print(f'Received: {msg.start_routing}, {msg.tb_id}, {msg.tb_goal}')
        self.tb_queue_job_task.append([msg.tb_id, np.array([msg.tb_goal.x, msg.tb_goal.y, msg.tb_goal.z])])
        print(f'All job tasks:')
        for task in self.tb_queue_job_task:
            print(task)
        print(f'-----')
        if msg.start_routing:
            low_level_route = []
            all_tb_id = []
            all_starts = []
            all_goals = []
            for tb_job in self.tb_queue_job_task:
                this_tb_route, start_pos = self.astar_planner(tb_job)
                low_level_route.append(this_tb_route)
                all_tb_id.append(tb_job[0])
                all_starts.append(start_pos)
                all_goals.append(tb_job[1])
                
                # Visualize individual path
                self.visualize_path(tb_job[0], start_pos, tb_job[1], this_tb_route)
            
            # Visualize all paths together
            self.visualize_all_paths(low_level_route, all_starts, all_goals)
            
            self.pub_tb_path(all_tb_id, low_level_route)
            self.tb_queue_job_task = []

    def is_valid_move(self, pos):
        return not np.any(np.all(np.abs(self.obstacles - pos) < 1e-6, axis=1))
    # def is_valid_move(self, pos):
    #     for obstacle in self.obstacles:
    #         if np.all(np.abs(obstacle - pos) < 1e-6):
    #             return False  # The position is too close to an obstacle
    #     return True  # The position is not close to any obstacle
    
    def astar_planner(self, tb_job): # Euclidean distance
        curr_tb_node_pos = next((tb["curr_pos"] for tb in self.tb_pos if tb["id"] == tb_job[0]), None)
        if curr_tb_node_pos is None:
            print(f"Error: No position found for tb_id {tb_job[0]}")
            return [], None

        start_pos = curr_tb_node_pos.copy()
        
        print(f'Start planning tb: {tb_job[0]} path...')
        print(f'Current position: {curr_tb_node_pos}')
        a_star_path = [curr_tb_node_pos]
        dis_per_grid = 3

        for step in range(10000):
            print(f'-----tb: {tb_job[0]}--step {step}------------')
            print(f'Current pos: {curr_tb_node_pos}')
            prop_move = [
                curr_tb_node_pos + np.array([dis_per_grid, 0, 0]),
                curr_tb_node_pos - np.array([dis_per_grid, 0, 0]),
                curr_tb_node_pos + np.array([0, dis_per_grid, 0]),
                curr_tb_node_pos - np.array([0, dis_per_grid, 0]),
            ]
            valid_moves = [move for move in prop_move if self.is_valid_move(move)]
            
            print(f'Valid moves:')
            for move in valid_moves:
                print(f'prop: {move}')
            print(f'tb_goal: {tb_job[1]}')

            if not valid_moves:
                print("No valid moves available. Path planning failed.")
                return a_star_path, start_pos

            distances = [np.linalg.norm(tb_job[1][:2] - move[:2]) for move in valid_moves]
            min_index = distances.index(min(distances))
            print(f'distances: {distances}')
            print(f'min index: {min_index}')

            curr_tb_node_pos = valid_moves[min_index]
            a_star_path.append(curr_tb_node_pos)
            print(f'New current pos: {curr_tb_node_pos}')
            
            if np.linalg.norm(tb_job[1][:2] - curr_tb_node_pos[:2]) < 1e-6:
                print("Finished path planning")
                return a_star_path, start_pos
        
        print("Warning: Maximum iterations reached without finding path")
        return a_star_path, start_pos

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
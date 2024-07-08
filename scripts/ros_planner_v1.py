#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import numpy as np
from mapf_isaac.msg import TbPathtoGoal, TbTask, Tbpose2D
from nav_msgs.msg import Odometry
import os
import yaml
from datetime import datetime
import time

from astar_planner import AstarPlanner
from visualizer import Visualizer

def load_uneven_astar_config():
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'uneven_astar.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config['costs']

class AstarPathPlanner(Node):
    def __init__(self):
        super().__init__('Astar_Path_Planner')
        self.config = load_uneven_astar_config()
        print(f"Costs: {self.config}")

        
        self.sub_task = self.create_subscription(TbTask, 'all_tb_job_task', self.sub_tb_job_callback, 10)
        self.pub_path_goal = self.create_publisher(TbPathtoGoal, 'TbPathtoGoal_top', 10)
        self.sub_tb_pos = self.create_subscription(Tbpose2D, 'tb_pose2d', self.sub_tbs_pos_cb, 10)
        
        self.tb_queue_job_task = []
        self.tb_pos = [
            {"id": i, "name": f"tb_{i}", "curr_pos": np.zeros(3), "initial": np.zeros(3)}
            for i in range(1, 4)
        ]
        
        self.path_think_step = [
            {"id": 1, "tstep": 0},
            {"id": 2, "tstep": 0},
            {"id": 3, "tstep": 0},
        ]

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
            [13.5, 16.5, 0], [13.5, 19.5, 0], [13.5, 22.5, 0],
            # frame bottom
            [16.5, -25.5, 0], [16.5, -22.5, 0], [16.5, -19.5, 0], [16.5, -16.5, 0],
            [16.5, -13.5, 0], [16.5, -10.5, 0], [16.5, -7.5, 0], [16.5, -4.5, 0],
            [16.5, -1.5, 0], [16.5, 1.5, 0], [16.5, 4.5, 0], [16.5, 7.5, 0],
            [16.5, 10.5, 0], [16.5, 13.5, 0], [16.5, 16.5, 0], [16.5, 19.5, 0],
            [16.5, 22.5, 0], [16.5, 25.5, 0], [16.5, 28.5, 0],
            # frame right
            [16.5, 28.5, 0], [13.5, 28.5, 0], [10.5, 28.5, 0], [7.5, 28.5, 0],
            [4.5, 28.5, 0], [1.5, 28.5, 0], [-1.5, 28.5, 0], [-4.5, 28.5, 0],
            [-7.5, 28.5, 0], [-10.5, 28.5, 0], [-13.5, 28.5, 0], [-16.5, 28.5, 0],
            [-19.5, 28.5, 0], [-22.5, 28.5, 0], [-25.5, 28.5, 0],
            # frame top
            [-16.5, 25.5, 0],  [-16.5, 22.5, 0],  [-16.5, 19.5, 0],  [-16.5, 16.5, 0],
            [-16.5, 13.5, 0],  [-16.5, 10.5, 0],  [-16.5, 7.5, 0],   [-16.5, 4.5, 0],
            [-16.5, 1.5, 0],   [-16.5, -1.5, 0],  [-16.5, -4.5, 0],  [-16.5, -7.5, 0],
            [-16.5, -10.5, 0], [-16.5, -13.5, 0], [-16.5, -16.5, 0], [-16.5, -19.5, 0],
            [-16.5, -22.5, 0], [-16.5, -25.5, 0], [-16.5, -28.5, 0],
            # frame left
            [-25.5, -28.5, 0], [-22.5, -28.5, 0], [-19.5, -28.5, 0], [-16.5, -28.5, 0],
            [-13.5, -28.5, 0], [-10.5, -28.5, 0], [-7.5, -28.5, 0], [-4.5, -28.5, 0],
            [-1.5, -28.5, 0], [1.5, -28.5, 0], [4.5, -28.5, 0], [7.5, -28.5, 0],
            [10.5, -28.5, 0], [13.5, -28.5, 0], [16.5, -28.5, 0], [19.5, -28.5, 0],
            [22.5, -28.5, 0], [25.5, -28.5, 0], [28.5, -28.5, 0],
        ])

        self.astar_planner = AstarPlanner(self.obstacles)
        self.visualizer = Visualizer()

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

    def sub_tb_job_callback(self, msg):
        self.tb_queue_job_task.append([msg.tb_id, np.array([msg.tb_goal.x, msg.tb_goal.y, msg.tb_goal.z])])
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
        all_path_steps = []
        
        for tb_job in self.tb_queue_job_task:
            path, start_pos, path_steps = self.astar_planner.plan(tb_job, self.tb_pos)
            if path:
                all_paths.append(path)
                all_tb_id.append(tb_job[0])
                all_starts.append(start_pos)
                all_goals.append(tb_job[1])
                all_path_steps.append(path_steps)
            else:
                print(f"Failed to find path for tb_id: {tb_job[0]}")
        
        #log overall planning
        self.astar_planner.log_overall_planning()
        
        # Visualizations
        for i, tb_id in enumerate(all_tb_id):
            self.visualizer.visualize_path(tb_id, all_starts[i], all_goals[i], all_paths[i])
            for step, (current, open_set, closed_set, g_score, f_score, came_from) in enumerate(all_path_steps[i]):
                self.visualizer.visualize_astar_path(tb_id, all_starts[i], all_goals[i], current, open_set, closed_set, g_score, f_score, came_from, step)
        
        if all_paths:
            print(f'All paths: ')
            for i in range(len(all_paths)):
                print(f'TB_{all_tb_id[i]}: {all_paths[i]}')
            
            sorted_indices = sorted(range(len(all_tb_id)), key=lambda k: all_tb_id[k])
            sorted_paths = [all_paths[i] for i in sorted_indices]
            sorted_starts = [all_starts[i] for i in sorted_indices]
            sorted_goals = [all_goals[i] for i in sorted_indices]
            sorted_tb_id = [all_tb_id[i] for i in sorted_indices]
            
            print(f'/*/*/*/*/*/*/*/*/*/*/*/')
            print(f'Sorted TB IDs: ')
            for i, tb_id in enumerate(sorted_tb_id):
                print(f'TB_{tb_id}: {sorted_paths[i]}')
            
            self.visualizer.visualize_all_paths(sorted_paths, sorted_starts, sorted_goals)
            self.pub_tb_path(sorted_tb_id, sorted_paths)
        else:
            print("No valid paths found for any robots")
        
        self.tb_queue_job_task = []

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
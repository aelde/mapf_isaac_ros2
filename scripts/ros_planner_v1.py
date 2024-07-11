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
import sys
from map.map import obstacles_1

from planner_control import PlannerControl
# from astar_planner import AstarPlanner
# from visualizer import Visualizer

def load_uneven_astar_config():
    config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'uneven_astar.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    return config['costs']

class RosPathPlanner(Node):
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

        self.obstacles = obstacles_1
        # self.obstacles = np.array([0,0,0])
        self.planner_control = PlannerControl(self.obstacles)
        # self.astar_planner = AstarPlanner(self.obstacles)
        # self.visualizer = Visualizer()

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

    # def plan_and_publish_paths(self):
    #     tasks = [(tb_job[0], tb_job[1]) for tb_job in self.tb_queue_job_task]
    #     all_paths = self.planner_control.plan_multiple_paths(tasks, self.tb_pos)

    #     if all_paths:
    #         print(f'All paths: ')
    #         for i, path in enumerate(all_paths):
    #             print(f'TB_{tasks[i][0]}: {path}')
            
    #         sorted_indices = sorted(range(len(tasks)), key=lambda k: tasks[k][0])
    #         sorted_paths = [all_paths[i] for i in sorted_indices]
    #         sorted_tb_id = [tasks[i][0] for i in sorted_indices]
            
    #         print(f'/*/*/*/*/*/*/*/*/*/*/*/')
    #         print(f'Sorted TB IDs: ')
    #         for i, tb_id in enumerate(sorted_tb_id):
    #             print(f'TB_{tb_id}: {sorted_paths[i]}')
            
    #         self.pub_tb_path(sorted_tb_id, sorted_paths)
    #     else:
    #         print("No valid paths found for any robots")
        
    #     self.tb_queue_job_task = []
    
    def plan_and_publish_paths(self):
        tasks = [(tb_job[0], tb_job[1]) for tb_job in self.tb_queue_job_task]
        all_paths = self.planner_control.plan_multiple_paths(tasks, self.tb_pos)

        if all_paths:
            print(f'All paths: ')
            for i, path in enumerate(all_paths):
                print(f'TB_{tasks[i][0]}: {path}')
            
            sorted_indices = sorted(range(len(tasks)), key=lambda k: tasks[k][0])
            sorted_paths = [all_paths[i] for i in sorted_indices]
            sorted_tb_id = [tasks[i][0] for i in sorted_indices]
            
            print(f'/*/*/*/*/*/*/*/*/*/*/*/')
            print(f'Sorted TB IDs: ')
            for i, tb_id in enumerate(sorted_tb_id):
                print(f'TB_{tb_id}: {sorted_paths[i]}')
            
            self.pub_tb_path(sorted_tb_id, sorted_paths)
        else:
            print("No valid paths found for any robots")
        
        self.tb_queue_job_task = []

def main(args=None):
    rclpy.init(args=args)
    path_planner = RosPathPlanner()
    print('RosPathPlanner node running')
    try:
        rclpy.spin(path_planner)
    except KeyboardInterrupt:
        print('RosPathPlanner node terminated')
    finally:
        path_planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
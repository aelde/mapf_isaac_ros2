#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray,Int32MultiArray
import numpy as np
from mapf_isaac.msg import TbPathtoGoal, TbTask, Tbpose2D
from nav_msgs.msg import Odometry
import os
import yaml
from datetime import datetime
import time
import sys
import glob
from dotenv import load_dotenv
from all_planner.ex_planner_control import PlannerControl
from all_planner.ex_me_visualizer import Visualizer
from all_planner.DICISION import CONV_NORMAL_TO_POS,CONV_POS_TO_NORMAL,WS_DIR,MAP_NAME,TOTAL_ROBOTS,OBSTACLES_MAP

load_dotenv()

def import_mapf_instance(filename):
    f = open(filename, 'r')
    rows, columns = [int(x) for x in f.readline().split(' ')]
    my_map = []
    for _ in range(rows):
        my_map.append([cell == '@' for cell in f.readline().strip()])
    f.close()
    return my_map 

def glob_map_files():
    config_dir = f'map/{MAP_NAME}.txt'
    config_d = os.path.join(WS_DIR, config_dir)
    map_files = glob.glob(config_d)
    if map_files:
        # print(f'Map files found: {map_files}')
        return import_mapf_instance(map_files[0])  # Use the first file found
    else:
        print("No map files found!")
        return None

class RosPathPlanner(Node):
    def __init__(self):
        super().__init__('Path_Planner')
        
        self.sub_task = self.create_subscription(TbTask, 'all_tb_job_task', self.sub_tb_job_callback, 10)
        self.pub_path_goal = self.create_publisher(TbPathtoGoal, 'TbPathtoGoal_top', 10)
        self.sub_tb_pos = self.create_subscription(Tbpose2D, 'tb_pose2d', self.sub_tbs_pos_cb, 10)
        self.sub_tb_is_moving = self.create_subscription(Int32MultiArray, 'robot_is_moving', self.sub_is_moving,10)
        
        # Subscriptions
        self.create_subscription(Odometry, 'tb_1_green/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_2_red/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_3_blue/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_4_sky/odom', self.tb_odom_cb, 10)
        
        self.tb_queue_job_task = []
        self.tb_pos = [
            {"id": i, "name": f"tb_{i}", "curr_pos": np.zeros(3),"start_pos": np.zeros(3), "goal_pos": None, "is_moving": None}
            for i in range(1, TOTAL_ROBOTS+1)
        ]
        self.is_start = [True for i in range(TOTAL_ROBOTS)]
        self.path_think_step = [
            {"id": 1, "tstep": 0},
            {"id": 2, "tstep": 0},
            {"id": 3, "tstep": 0},
        ]
        self.all_start = [{f'tb_{i+1}': []} for i in range(len(self.tb_pos))]
        self.map = glob_map_files()
        if self.map is None:
            self.get_logger().error('No map file found. Exiting.')
            rclpy.shutdown()
            sys.exit(1)
        
        # self.planner_control = PlannerControl(self.map)
        # self.astar_planner = AstarPlanner(self.map)
        # self.visualizer = Visualizer()
    
    def tb_odom_cb(self, msg):
        pass  
    def sub_is_moving(self, msg):
        is_moving = [*msg.data]
        for i,j in enumerate(self.tb_pos):
            j["is_moving"] = is_moving[i]

        # for i in self.tb_pos:print(i)
        
    def sub_tbs_pos_cb(self, msg):
        for i in range(len(self.tb_pos)):
            if self.tb_pos[i]["id"]== msg.tb_id:
                self.tb_pos[i]["curr_pos"] = np.array([msg.tb_pos.x, msg.tb_pos.y, 0.0])
                if self.is_start[i]:
                    self.tb_pos[i]["start_pos"] = np.array([msg.tb_pos.x, msg.tb_pos.y, 0.0])
                    self.is_start[i] = False
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
        if (msg.tb_id not in range(1,TOTAL_ROBOTS+1)) and (msg.tb_id != -1):
            self.get_logger().error(f'tb_id:{msg.tb_id} dont EXITTTT...')
            self.get_logger().error(f'bye EIEIE')
            self.destroy_node()
            rclpy.shutdown()
        self.tb_queue_job_task.append([msg.tb_id, np.array([msg.tb_goal.x, msg.tb_goal.y, msg.tb_goal.z])])
        print(f'tb_queue: \n{self.tb_queue_job_task}')
        print(f'****')
        
        a = []
        for i in range(len(self.tb_queue_job_task)): 
            if self.tb_queue_job_task[i][0] in range(1,TOTAL_ROBOTS+1): 
                for j in self.tb_pos:
                    if j["id"] == self.tb_queue_job_task[i][0]:
                        a.append(j["is_moving"])
                        
        if msg.start_routing and sum(a) == 0:
            open_start = [i["start_pos"] for i in self.tb_pos]
            print(f'open_start: \n{open_start}')
            for i in range(len(self.tb_queue_job_task)): 
                if self.tb_queue_job_task[i][0] in range(1,TOTAL_ROBOTS+1): 
                    for j in self.tb_pos:
                        if j["id"] == self.tb_queue_job_task[i][0]:
                            j["goal_pos"] = self.tb_queue_job_task[i][1] 
                            open_start = [i for i in open_start if not np.array_equal(i, j["start_pos"])]
                            print(f'open_start_{i}_specific: \n{open_start}')
                            # j["start_pos"] = j["goal_pos"]
                            for k in self.all_start:
                                if k == j["name"]: 
                                    k.append(j["start_pos"])
                                    print(f'all_start: {k}')
                elif self.tb_queue_job_task[i][0] == -1:
                    # calculate norm from tb_queue_job_task[0] - tb_queue_job_task[n]
                    print([round(np.linalg.norm(self.tb_queue_job_task[i][1][:2]-open_start[j][:2]),2) for j in range(len(open_start))])
                    minn_norm = round(min(np.linalg.norm(self.tb_queue_job_task[i][1][:2]-open_start[j][:2]) for j in range(len(open_start))),2)
                    print(f'tb_queue_{i}: {self.tb_queue_job_task[i][1][:2]} , {CONV_POS_TO_NORMAL(self.tb_queue_job_task[i][1][:2])}')
                    print(f'minn_norm: {minn_norm}')
                    # minnn = min(self.tb_pos, key=lambda x: np.linalg.norm(self.tb_queue_job_task[i][1][:2] - np.array(x['start_pos'][:2])))
                    # print(f'minnn: \n{minnn}')
                    for j in self.tb_pos:
                        if minn_norm == round(np.linalg.norm(self.tb_queue_job_task[i][1][:2] - j["start_pos"][:2]), 2):
                            j["goal_pos"] = self.tb_queue_job_task[i][1] 
                            print(f'j: {j["start_pos"]} , {CONV_POS_TO_NORMAL(j["start_pos"])}')
                            open_start = [i for i in open_start if not np.array_equal(i, j["start_pos"])]
                            print(f'open_start_{i}_any: \n{open_start}')
                            # j["start_pos"] = j["goal_pos"]
                            for k in self.all_start:
                                if k == j["name"]: 
                                    k.append(j["start_pos"])
                                    print(f'all_start: {k}')
                else: print('ORTHER????')
            for i in self.tb_pos:
                if i["goal_pos"] is not None: continue
                else: i["goal_pos"] = i["start_pos"]
            
            print('Ending sub_tb_job_callback...')
            for i in self.tb_pos: print(i)
            
            print('allstart...')
            for i in self.all_start: print(i)
            
            self.plan_and_publish_paths()
        
        else: self.tb_queue_job_task = self.tb_queue_job_task
    
    def plan_and_publish_paths(self):
        # tasks = [(tb_job[0], tb_job[1]) for tb_job in self.tb_queue_job_task]
        tasks = [(i["id"], i["goal_pos"]) for i in self.tb_pos]
        print(f'Tasks: {tasks}')
        print(f'TB Pos:')
        for tb in self.tb_pos: print(tb)
        print(f'************************************')
        starts = [(self.tb_pos[i]["start_pos"][:2]) for i in range(len(self.tb_pos))]
        goals = [(self.tb_pos[i]["goal_pos"][:2]) for i in range(len(self.tb_pos))]
        # normal_start = [convert_pos_to_normal_p(starts[i]) for i in range(len(starts))]
        normal_start = [CONV_POS_TO_NORMAL(starts[i]) for i in range(len(starts))]
        normal_goals = [CONV_POS_TO_NORMAL(goals[i]) for i in range(len(goals))]
        
        # set new start
        for i in self.tb_pos: i["start_pos"] = i["goal_pos"] 
        print(f'new_start: \n{self.tb_pos}')
        
        print(f'self.tb_pos len: {len(self.tb_pos)}')
        print(f'Starts: \n{starts}')
        print(f'Gaols: \n{goals}')
        print(f'Converted Starts: \n{normal_start}')
        print(f'Converted Goals: \n{normal_goals}')
        print()
        
        solution = PlannerControl().plan_paths(self.map, normal_start, normal_goals)
        paths, nodes_gen, nodes_exp = solution[:3]
        all_paths = paths # 0 - n
        all_paths_conv = [np.array([CONV_NORMAL_TO_POS(point) for point in path]) for path in paths] 
        print(f'All paths: ')
        for i in all_paths[0]: print(i)
        # print(all_paths)
        print()
        print(f'All paths converted: ')
        # print(all_paths_conv)
        for i in all_paths_conv[0]: print(i)
        print()
        
        # visualize each path
        for i, path in enumerate(all_paths_conv):
            Visualizer.visualize_path(OBSTACLES_MAP, i+1, starts[i], goals[i], path,MAP_NAME)
        # visualize all paths
        Visualizer.visualize_all_paths(OBSTACLES_MAP, all_paths_conv, starts, goals,MAP_NAME)
        Visualizer.visualize_all_paths_2(OBSTACLES_MAP, all_paths_conv, starts, goals,MAP_NAME)
        
        # visualize animate
        PlannerControl.show_animation(self.map, normal_start, normal_goals, all_paths)
        
        if all_paths_conv:
            print(f'All paths: ')
            for i, path in enumerate(all_paths_conv):
                print(f'TB_{tasks[i][0]}: {path}')
            
            sorted_indices = sorted(range(len(tasks)), key=lambda k: tasks[k][0])
            # sorted_paths = [all_paths_conv[i] for i in sorted_indices]
            sorted_tb_id = [tasks[i][0] for i in sorted_indices]
            
            # now all_paths_conv is 2d array , must convert to 3d array
            all_paths_conv_3d = [np.hstack((arr, np.zeros((arr.shape[0], 1)))) for arr in all_paths_conv]
            for i, path in enumerate(all_paths_conv_3d):
                print(f'TB_{tasks[i][0]}: {path}')
            
            self.pub_tb_path(sorted_tb_id, all_paths_conv_3d)
            # self.pub_tb_path(sorted_tb_id, sorted_paths)
        else:
            print("No valid paths found for any robots")
            
        PlannerControl.show_animation(self.map, normal_start, normal_goals, all_paths)
        
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
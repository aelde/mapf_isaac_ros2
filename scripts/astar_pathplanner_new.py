#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
import numpy as np
from mapf_isaac.msg import TbPathtoGoal,TbTask,Tbpose2D
from nav_msgs.msg import Odometry


class Testpub(Node):
    def __init__(self):
        super().__init__('Astar_Path_Planner')
        self.sub = self.create_subscription(
            TbTask, 'all_tb_job_task', self.sub_tb_job_callback, 10)
        # self.pubtb_path = self.create_publisher(Float32MultiArray, 'all_tb_path', 10)
        self.pubtb_path_goal = self.create_publisher(TbPathtoGoal, 'TbPathtoGoal_top', 10)
        # self.sub_initial_tb_pos = self.create_subscription(TbTask,'tb_initial_pos',self.sub_initial_pos,10)
        self.sub_tb_pos = self.create_subscription(Tbpose2D,'tb_pose2d',self.sub_tbs_pos_cb,10)
        # self.sub_tb_pos = self.create_subscription(TbTask,'all_tb_pos',self.sub_tbs_pos_cb,10)
        
        self.sub_tb_pos = self.create_subscription(Odometry, 'tb_1_green/odom', self.tb_1_odom_cb, 10)
        self.sub_tb_pos = self.create_subscription(Odometry, 'tb_2_red/odom', self.tb_2_odom_cb, 10)
        self.sub_tb_pos = self.create_subscription(Odometry, 'tb_3_blue/odom', self.tb_3_odom_cb, 10)
        
        self.tb_queue_job_task = []
        self.low_level_route = []
        self.all_tb_id = []
        
        self.initial_tb1 = np.array([0,0,0])
        self.initial_tb2 = np.array([0,0,0])
        
        self.tb_pos = [
            {"id": 1, 
             "name": "tb_1",
             "curr_pos": np.array([0,0,0]), 
             "initial": np.array([0,0,0]),
             }, 
            {"id": 2,
             "name": "tb_2",
             "curr_pos": np.array([0,0,0]), 
             "initial": np.array([0,0,0]),
             }, 
            {"id": 3,
             "name": "tb_3",
             "curr_pos": np.array([0,0,0]), 
             "initial": np.array([0,0,0]),
             }, 
            # {"id": 3, "name": "tb_3", "color": np.array([0,0,1])},  # Blue color for the third link
            # Add more job entries as needed
        ]
    # def sub_initial_pos(self,msg):
    #     tb_id = msg.tb_id
    #     cur_pos = msg.tb_goal
    #     start_routing = msg.start_routing
    #     for i in self.tb_pos:
    #         if i["id"] == tb_id: 
    #             i["initial"] = np.array([cur_pos.x,cur_pos.y,0.0])
    #             print(f'tb{tb_id}, pos:{i["initial"]}')
    def sub_tbs_pos_cb(self,msg):
        # print(f'msg: {msg}')
        tb_id = msg.tb_id
        cur_pos = msg.tb_pos
        # start_routing = msg.start_routing
        for i in self.tb_pos:
            if i["id"] == tb_id: 
                i["curr_pos"] = np.array([cur_pos.x,cur_pos.y,0.0])
                # print(f'pos-> id:{i["id"]} : {i["curr_pos"]}')
    #     print()
        # print(f'self tb pos : {self.tb_pos}')
    def tb_1_odom_cb(self,msg):
        position = msg.pose.pose.position
        # self.tb_pos[0]["curr_pos"] = np.array([position.x,position.y,0.0]) + self.tb_pos[0]["initial"]
    def tb_2_odom_cb(self,msg):
        position = msg.pose.pose.position
        # self.tb_pos[1]["curr_pos"] = np.array([position.x,position.y,0.0]) + self.tb_pos[1]["initial"]
    def tb_3_odom_cb(self,msg):
        position = msg.pose.pose.position
        # self.tb_pos[2]["curr_pos"] = np.array([position.x,position.y,0.0]) + self.tb_pos[2]["initial"]
        # print(f'tb1: {self.tb_pos[0]["curr_pos"]}',end=' ')
        # print(f'tb2: {self.tb_pos[1]["curr_pos"]}',end=' ')
        # print(f'tb3: {self.tb_pos[2]["curr_pos"]}')
        # print()

    def pub_tb_path(self, tb_ids, paths):
        for tb_id, path in zip(tb_ids, paths):
            msg = Float32MultiArray()
            msg_path = TbPathtoGoal()
            msg.data = []

            for point in path:
                msg.data.extend(point)
            
            dim0 = MultiArrayDimension()
            dim0.label = "points"
            dim0.size = len(path)
            dim0.stride = len(msg.data)
            dim1 = MultiArrayDimension()
            dim1.label = "coordinates"
            dim1.size = 3  # assuming 3D coordinates
            dim1.stride = 3
            
            msg.layout.dim = [dim0, dim1]
            # self.pubtb_path.publish(msg)

            msg_path.tb_id = tb_id
            msg_path.listofpath = msg
            self.pubtb_path_goal.publish(msg_path)

            print(f'Published path for tb_id: {tb_id}')
            print(f'Path: ')
            for i in range(len(path)):
                print(f'{path[i]}')

    def sub_tb_job_callback(self, msg):
        print(f'Received: {msg.start_routing}, {msg.tb_id}, {msg.tb_goal}')
        self.tb_queue_job_task.append([msg.tb_id, np.array([msg.tb_goal.x, msg.tb_goal.y, msg.tb_goal.z])])
        print(f'All job tasks:')
        for i in self.tb_queue_job_task:
            print(i)
        print(f'-----')
        if msg.start_routing:
            self.low_level_route = []
            self.all_tb_id = []
            for tb_job in self.tb_queue_job_task:
                this_tb_route = self.astar_planner(tb_job)
                self.low_level_route.append(this_tb_route)
                self.all_tb_id.append(tb_job[0])
            self.pub_tb_path(self.all_tb_id, self.low_level_route)
            self.tb_queue_job_task = []

    def astar_planner(self, tb_job):
        curr_tb_node_pos = []
        print(f'Start planning tb: {tb_job[0]} path...')
        for i in self.tb_pos:
            if i["id"] == tb_job[0]: 
                curr_tb_node_pos = i["curr_pos"]

        # curr_tb_node_pos = np.array([-7.5, -25.5, 0.0])
        print(f'cuurr : {curr_tb_node_pos}')
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
            print(f'Proposed moves:')
            for move in prop_move:
                print(f'prop: {move}')
            print(f'tb_goal: {tb_job[1]}')

            distances = [np.linalg.norm(tb_job[1][:2] - move[:2]) for move in prop_move]
            min_index = distances.index(min(distances))
            print(f'distances: {distances}')
            print(f'min index: {min_index}')

            a_star_path.append(prop_move[min_index])
            curr_tb_node_pos = prop_move[min_index]
            print(f'Current pos: {curr_tb_node_pos}')
            
            if np.array_equal(tb_job[1], curr_tb_node_pos):
                print("Finished path planning")
                return a_star_path
        
        return a_star_path

def main(args=None):
    rclpy.init(args=args)
    my_pub = Testpub()
    print('Test node running')
    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        print('Test node terminated')
    finally:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

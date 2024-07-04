#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from mapf_isaac.msg import TbPathtoGoal,TbTask,Tbpose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import time

class PathFollowing(Node):
    def __init__(self):
        super().__init__('multi_robot_cmd_vel_publisher')
        self.sub_tb_pos = self.create_subscription(Odometry, 'tb_1_green/odom', self.tb_1_odom_cb, 10)
        self.sub_tb_pos = self.create_subscription(Odometry, 'tb_2_red/odom', self.tb_2_odom_cb, 10)
        self.sub_tb_pos = self.create_subscription(Odometry, 'tb_3_blue/odom', self.tb_3_odom_cb, 10)
        self.sub_tb2d_pos = self.create_subscription(Tbpose2D,'tb_pose2d',self.sub_tb2d_pos,10)
        self.ros_sub = self.create_subscription(TbPathtoGoal, "TbPathtoGoal_top", self.tb_path_receive, 10)
        
         # Create publishers for each robot
        self.robots = ['robot1', 'robot2', 'robot3']
        self.pub = {}
        self.robot_pose2d = {
            'robot1': [0, 0, 0],
            'robot2': [0, 0, 0],
            'robot3': [0, 0, 0],
        }
        self.robot_path = {
            'robot1': None,
            'robot2': None,
            'robot3': None,
        }
        self.robot1_is_at_goal = False
        self.robot1_cur_goal = None
        self.robot2_is_at_goal = False
        self.robot2_cur_goal = None
        self.robot3_is_at_goal = False
        self.robot3_cur_goal = None
        
        self.start_time = time.time()
        self.end_time_r1 = None
        self.end_time_r2 = None
        self.end_time_r3 = None
        
        self.robot1_time = None
        self.robot2_time = None
        self.robot3_time = None
                
        for robot in self.robots:
            topic = f'/{robot}/cmd_vel'
            self.pub[robot] = self.create_publisher(Twist, topic, 10)
            self.get_logger().info(f'Publisher created for {topic}')
        
        # Timer to publish cmd_vel periodically
        self.timer = self.create_timer(0.5, self.publish_cmd_vel)
        
        self.test = []
    def tb_1_odom_cb(self, msg):
        pass
    def tb_2_odom_cb(self, msg):
        pass
    def tb_3_odom_cb(self, msg):
        pass
    def sub_tb2d_pos(self, msg):
        # print(f'msg: {msg}')
        # print(f'msg.tb_id: {msg.tb_id}')
        # print(f'msg.tb_pos: {msg.tb_pos.x}')
        # print(f'msg.tb_pos: {msg.tb_pos.y}')
        # print(f'msg.tb_pos: {msg.tb_pos.theta}')
        
        for i in self.robot_pose2d:
            if i == f'robot{msg.tb_id}':
                # print(f'i: {i}')
                # self.robot_pose2d[i] = [round(msg.tb_pos.x,2), round(msg.tb_pos.y,2), msg.tb_pos.theta]
                self.robot_pose2d[i] = [msg.tb_pos.x, msg.tb_pos.y, msg.tb_pos.theta]

    def publish_cmd_vel(self):
        goal_p = None
        for i in self.robot_path:
            if i == 'robot1':
                if self.robot_path[i] is not None:
                    length = len(self.robot_path[i])
                    if self.robot1_is_at_goal:
                        self.robot1_is_at_goal = False
                        self.robot_path[i].pop(0)
                        print(f'robot_path: {self.robot_path[i]}')
                    print(f'robot_path: {self.robot_path[i]}')
                    # to avoid index out of range err
                    if len(self.robot_path[i]) == 0:
                        self.robot_path[i] = None
                        self.end_time_r1 = time.time()
                        self.robot1_time = self.end_time_r1 - self.start_time
                        print(f'Execution time R1: {self.robot1_time} sec')
                        print(f'*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/')
                        break
                    self.robot1_cur_goal = np.array(self.robot_path[i][0][:2])
            if i == 'robot2':
                if self.robot_path[i] is not None:
                    length = len(self.robot_path[i])
                    if self.robot2_is_at_goal:
                        self.robot2_is_at_goal = False
                        self.robot_path[i].pop(0)
                        print(f'robot_path: {self.robot_path[i]}')
                    print(f'robot_path: {self.robot_path[i]}')
                    # to avoid index out of range err
                    if len(self.robot_path[i]) == 0:
                        self.robot_path[i] = None
                        self.end_time_r2 = time.time()
                        self.robot2_time = self.end_time_r2 - self.start_time
                        print(f'Execution time R2: {self.robot2_time} sec')
                        print(f'*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/')
                        break
                    self.robot2_cur_goal = np.array(self.robot_path[i][0][:2])
            if i == 'robot3':
                if self.robot_path[i] is not None:
                    length = len(self.robot_path[i])
                    if self.robot3_is_at_goal:
                        self.robot3_is_at_goal = False
                        self.robot_path[i].pop(0)
                        print(f'robot_path: {self.robot_path[i]}')
                    print(f'robot_path: {self.robot_path[i]}')
                    # to avoid index out of range err
                    if len(self.robot_path[i]) == 0:
                        self.robot_path[i] = None
                        self.end_time_r3 = time.time()
                        self.robot3_time = self.end_time_r3 - self.start_time
                        print(f'Execution time R3: {self.robot3_time} sec')
                        print(f'*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/')
                        break
                    self.robot3_cur_goal = np.array(self.robot_path[i][0][:2])
                
        if self.robot1_cur_goal is None:
             robot_1_cmd_vel = Twist()
             robot_1_cmd_vel.linear.x = 0.0  
             robot_1_cmd_vel.linear.y = 0.0  
        else:
            robot_1_cmd_vel = self.robot_1_cmd_vel(self.robot1_cur_goal)
        if self.robot1_cur_goal is None:
             robot_2_cmd_vel = Twist()
             robot_2_cmd_vel.linear.x = 0.0  
             robot_2_cmd_vel.linear.y = 0.0  
        else:
            robot_2_cmd_vel = self.robot_2_cmd_vel(self.robot2_cur_goal)
        if self.robot3_cur_goal is None:
             robot_3_cmd_vel = Twist()
             robot_3_cmd_vel.linear.x = 0.0  
             robot_3_cmd_vel.linear.y = 0.0  
        else:
            robot_3_cmd_vel = self.robot_3_cmd_vel(self.robot3_cur_goal)
        for robot, publisher in self.pub.items():
            # print(f'self.pub: {self.pub}')
            if robot == 'robot1':
                publisher.publish(robot_1_cmd_vel)
            if robot == 'robot2':
                publisher.publish(robot_2_cmd_vel)
            if robot == 'robot3':
                publisher.publish(robot_3_cmd_vel)
    def robot_1_cmd_vel(self,goal_p):
        msg_1 = Twist()
        # print(f'this is robot_1_cmd_vel')
        print(f'1111111111111111111')
        print(f'robot1_time: {self.robot1_time}')
        curr_p = np.array(self.robot_pose2d['robot1'][:2])
        if np.allclose(curr_p, goal_p, atol=1e-1):  # Adjust atol as needed for precision
            msg_1.linear.x = 0.0
            msg_1.angular.z = 0.0
            self.robot1_is_at_goal = True
            print(f'Goal reached for robot1111111')

        else:        
            print(f'goal_p: {goal_p}')
            print(f'curr_p: {curr_p}')

            a = goal_p - curr_p
            print(f'a: {a}')
            rot_goal = np.where(a == 0, 0.00001, a)
            # print(f'np.where: {rot_goal}')
            rot_goal = np.arctan2(rot_goal[1], rot_goal[0])  # Using arctan2 for correct angle calculation
            rot_goal_deg = np.degrees(rot_goal)
            # print(f'arctan2: {rot_goal_deg}')
                
            
            robot_1_angu = self.robot_pose2d['robot1'][2]
            # print(f'robot_1_angu: {robot_1_angu}, rot_goal: {rot_goal}')
            rot_robot_goal = rot_goal_deg - robot_1_angu
            print(f'rot_robot_goal: {rot_robot_goal}')
            msg_1.angular.z = 0.01 * rot_robot_goal  # Adjust angular velocity to turn towards the goal
            print(f'msg_1.angular.z: {msg_1.angular.z}')
            
            msg_1.linear.x = 0.0  # Linear velocity can be adjusted as needed
            
            if abs(rot_robot_goal) < 10:
                msg_1.linear.x = 0.03   # Adjust linear velocity to move towards the goal
                # msg_1.linear.x = 0.03 * np.linalg.norm(a)  # Adjust linear velocity to move towards the goal
                print(f'msg_1.linear.x: {msg_1.linear.x}')
                print(f'np.linalg.norm(a): {np.linalg.norm(a)}')
        # self.pub['robot1'].publish(msg_1)
        return msg_1        
    
    def robot_2_cmd_vel(self,goal_p):
        msg_2 = Twist()
        # print(f'this is robot_2_cmd_vel')
        print(f'2222222222222222222')
        print(f'robot2_time: {self.robot2_time}')
        curr_p = np.array(self.robot_pose2d['robot2'][:2])
        if np.allclose(curr_p, goal_p, atol=1e-1):  # Adjust atol as needed for precision
            msg_2.linear.x = 0.0
            msg_2.angular.z = 0.0
            self.robot2_is_at_goal = True
            print(f'Goal reached for robot2222222')
        else:        
            # print(f'goal_p: {goal_p}')
            # print(f'curr_p: {curr_p}')

            a = goal_p - curr_p
            print(f'a: {a}')
            rot_goal = np.where(a == 0, 0.00001, a)
            # print(f'np.where: {rot_goal}')
            rot_goal = np.arctan2(rot_goal[1], rot_goal[0])  # Using arctan2 for correct angle calculation
            rot_goal_deg = np.degrees(rot_goal)
            # print(f'arctan2: {rot_goal_deg}')
                
            
            robot_1_angu = self.robot_pose2d['robot2'][2]
            # print(f'robot_1_angu: {robot_1_angu}, rot_goal: {rot_goal}')
            rot_robot_goal = rot_goal_deg - robot_1_angu
            print(f'rot_robot_goal: {rot_robot_goal}')
            msg_2.angular.z = 0.01 * rot_robot_goal  # Adjust angular velocity to turn towards the goal
            print(f'msg_2.angular.z: {msg_2.angular.z}')
            
            msg_2.linear.x = 0.0  # Linear velocity can be adjusted as needed
            
            if abs(rot_robot_goal) < 10:
                msg_2.linear.x = 0.03   # Adjust linear velocity to move towards the goal
                # msg_2.linear.x = 0.03 * np.linalg.norm(a)  # Adjust linear velocity to move towards the goal
                print(f'msg_2.linear.x: {msg_2.linear.x}')
                print(f'np.linalg.norm(a): {np.linalg.norm(a)}')
        return msg_2
    
    def robot_3_cmd_vel(self,goal_p):
        msg_3 = Twist()
        # print(f'this is robot_3_cmd_vel')
        print(f'3333333333333333333')
        print(f'robot3_time: {self.robot3_time}')
        curr_p = np.array(self.robot_pose2d['robot3'][:2])
        if np.allclose(curr_p, goal_p, atol=1e-1):  # Adjust atol as needed for precision
            msg_3.linear.x = 0.0
            msg_3.angular.z = 0.0
            self.robot3_is_at_goal = True
            print(f'Goal reached for robot333333')
        else:        
            # print(f'goal_p: {goal_p}')
            # print(f'curr_p: {curr_p}')

            a = goal_p - curr_p
            print(f'a: {a}')
            rot_goal = np.where(a == 0, 0.00001, a)
            # print(f'np.where: {rot_goal}')
            rot_goal = np.arctan2(rot_goal[1], rot_goal[0])  # Using arctan2 for correct angle calculation
            rot_goal_deg = np.degrees(rot_goal)
            # print(f'arctan2: {rot_goal_deg}')
                
            
            robot_1_angu = self.robot_pose2d['robot3'][2]
            # print(f'robot_1_angu: {robot_1_angu}, rot_goal: {rot_goal}')
            rot_robot_goal = rot_goal_deg - robot_1_angu
            print(f'rot_robot_goal: {rot_robot_goal}')
            msg_3.angular.z = 0.01 * rot_robot_goal  # Adjust angular velocity to turn towards the goal
            print(f'msg_3.angular.z: {msg_3.angular.z}')
            
            msg_3.linear.x = 0.0  # Linear velocity can be adjusted as needed
            
            if abs(rot_robot_goal) < 10:
                msg_3.linear.x = 0.03  # Adjust linear velocity to move towards the goal
                # msg_3.linear.x = 0.03 * np.linalg.norm(a)  # Adjust linear velocity to move towards the goal
                print(f'msg_3.linear.x: {msg_3.linear.x}')
                print(f'np.linalg.norm(a): {np.linalg.norm(a)}')
        # self.pub['robot3'].publish(msg_3)
        return msg_3        
    
    def tb_path_receive(self, msg):
        tb_id = msg.tb_id
        data = msg.listofpath.data
        dim0_size = msg.listofpath.layout.dim[0].size
        dim1_size = msg.listofpath.layout.dim[1].size
        re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
        # print(f'len_tb_path: {len(re_tb_path)}')
        # print(f'tb_path: {re_tb_path}')
        for i in self.robot_path:
            if i == f'robot{tb_id}':
                self.robot_path[i] = re_tb_path
                # print(f'{i} :: robot_path:')
                for i in self.robot_path[i]: pass
                    # print(i)
def main(args=None):
    rclpy.init(args=args)
    my_pub = PathFollowing()
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

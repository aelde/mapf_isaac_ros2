#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from mapf_isaac.msg import TbPathtoGoal, TbTask, Tbpose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import time

class PathFollowing(Node):
    def __init__(self):
        super().__init__('multi_robot_cmd_vel_publisher')
        
        self.robots = ['robot1', 'robot2', 'robot3']
        self.robot_pose2d = {robot: [0, 0, 0] for robot in self.robots}
        self.robot_path = {robot: None for robot in self.robots}
        self.robot_goals_reached = {robot: False for robot in self.robots}
        self.robot_cur_goal = {robot: None for robot in self.robots}
        self.start_time = time.time()
        self.end_time = {robot: None for robot in self.robots}
        self.robot_time = {robot: None for robot in self.robots}
        
        # Subscriptions
        self.create_subscription(Odometry, 'tb_1_green/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_2_red/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_3_blue/odom', self.tb_odom_cb, 10)
        self.create_subscription(Tbpose2D, 'tb_pose2d', self.sub_tb2d_pos, 10)
        self.create_subscription(TbPathtoGoal, "TbPathtoGoal_top", self.tb_path_receive, 10)
        
        # Publishers
        self.pub = {robot: self.create_publisher(Twist, f'/{robot}/cmd_vel', 10) for robot in self.robots}
        
        # Timer to publish cmd_vel periodically
        self.timer = self.create_timer(0.5, self.publish_cmd_vel)

    def tb_odom_cb(self, msg):
        pass  # Implement if necessary

    def sub_tb2d_pos(self, msg):
        robot_id = f'robot{msg.tb_id}'
        self.robot_pose2d[robot_id] = [msg.tb_pos.x, msg.tb_pos.y, msg.tb_pos.theta]

    def publish_cmd_vel(self):
        for robot in self.robots:
            if self.robot_path[robot] is not None:
                if self.robot_goals_reached[robot]:
                    self.robot_goals_reached[robot] = False
                    self.robot_path[robot].pop(0)

                if len(self.robot_path[robot]) == 0:
                    self.robot_path[robot] = None
                    self.end_time[robot] = time.time()
                    self.robot_time[robot] = self.end_time[robot] - self.start_time
                    print(f'Execution time {robot}: {self.robot_time[robot]} sec')
                    print(f'*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/')
                    continue
                
                self.robot_cur_goal[robot] = np.array(self.robot_path[robot][0][:2])

            cmd_vel = self.calculate_cmd_vel(robot)
            self.pub[robot].publish(cmd_vel)
    
    def calculate_cmd_vel(self, robot):
        if self.robot_cur_goal[robot] is None:
            return Twist()
        
        goal_p = self.robot_cur_goal[robot]
        curr_p = np.array(self.robot_pose2d[robot][:2])
        if np.allclose(curr_p, goal_p, atol=1e-1):
            self.robot_goals_reached[robot] = True
            return Twist()

        a = goal_p - curr_p
        rot_goal = np.arctan2(a[1], a[0])
        rot_goal_deg = np.degrees(rot_goal)
        robot_angu = self.robot_pose2d[robot][2]
        rot_robot_goal = rot_goal_deg - robot_angu
        
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.01 * rot_robot_goal
        if abs(rot_robot_goal) < 10:
            cmd_vel.linear.x = 0.03
        
        return cmd_vel

    def tb_path_receive(self, msg):
        tb_id = msg.tb_id
        data = msg.listofpath.data
        dim0_size = msg.listofpath.layout.dim[0].size
        dim1_size = msg.listofpath.layout.dim[1].size
        re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
        robot_id = f'robot{tb_id}'
        self.robot_path[robot_id] = re_tb_path

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

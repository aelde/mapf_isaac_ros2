#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from mapf_isaac.msg import TbPathtoGoal, TbTask, Tbpose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import time
from datetime import datetime
import os
from all_planner.DICISION import WS_DIR, TOTAL_ROBOTS

class PathFollowing(Node):
    def __init__(self):
        super().__init__('multi_robot_cmd_vel_publisher')
        
        # self.robots = ['robot1', 'robot2', 'robot3']
        self.robots = [f'robot{i}'for i in range(1,TOTAL_ROBOTS+1)]
        self.robot_pose2d = {robot: [0, 0, 0] for robot in self.robots}
        self.robot_path = {robot: None for robot in self.robots}
        self.robot_goals_reached = {robot: False for robot in self.robots}
        self.robot_cur_goal = {robot: None for robot in self.robots}
        self.robot_time = {robot: {'start': None, 'end': None, 'duration': None} for robot in self.robots}
        self.robot_movements = {robot: {'straight': 0, 'rotate': 0} for robot in self.robots}
        
        self.current_timestep = 0
        self.max_timesteps = 0
        self.robot_timesteps = {robot: [] for robot in self.robots}
        self.robot_current_timestep_index = {robot: 0 for robot in self.robots}
        self.all_robots_reached_current_timestep = False
        
        self.all_robots_completed = False
        self.all_robots_exec_time = []
        
        self.create_subscription(Odometry, 'tb_1_green/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_2_red/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_3_blue/odom', self.tb_odom_cb, 10)
        self.create_subscription(Tbpose2D, 'tb_pose2d', self.sub_tb2d_pos, 10)
        self.create_subscription(TbPathtoGoal, "TbPathtoGoal_top", self.tb_path_receive, 10)
        
        self.pub = {robot: self.create_publisher(Twist, f'/{robot}/cmd_vel', 10) for robot in self.robots}
        
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10 Hz update rate

    def tb_odom_cb(self, msg):
        pass

    def sub_tb2d_pos(self, msg):
        robot_id = f'robot{msg.tb_id}'
        self.robot_pose2d[robot_id] = [msg.tb_pos.x, msg.tb_pos.y, msg.tb_pos.theta]
    
    def publish_cmd_vel(self):
        if not all(self.robot_path.values()):
            return

        all_completed = True
        all_reached_current_timestep = True

        for robot in self.robots:
            if self.robot_current_timestep_index[robot] < len(self.robot_timesteps[robot]):
                all_completed = False
                current_goal = self.robot_timesteps[robot][self.robot_current_timestep_index[robot]]
                
                if not np.array_equal(self.robot_cur_goal[robot], current_goal):
                    self.robot_cur_goal[robot] = np.array(current_goal)
                    self.robot_goals_reached[robot] = False

                if not self.robot_goals_reached[robot]:
                    all_reached_current_timestep = False
                    cmd_vel = self.calculate_cmd_vel(robot)
                    self.pub[robot].publish(cmd_vel)
                else:
                    self.pub[robot].publish(Twist())  # Stop the robot

        if all_reached_current_timestep and not all_completed:
            self.current_timestep += 1
            for robot in self.robots:
                if self.robot_current_timestep_index[robot] < len(self.robot_timesteps[robot]):
                    self.robot_current_timestep_index[robot] += 1
                    self.robot_goals_reached[robot] = False

        if all_completed and not self.all_robots_completed:
            self.all_robots_completed = True
            self.log_total_time()

    def calculate_cmd_vel(self, robot):
        if self.robot_cur_goal[robot] is None:
            return Twist()
        
        goal_p = self.robot_cur_goal[robot]
        curr_p = np.array(self.robot_pose2d[robot][:2])
        
        # check reached the goal
        if np.linalg.norm(curr_p - goal_p) < 0.1:  # 10cm threshold
            self.robot_goals_reached[robot] = True
            return Twist()

        # calculate angle to goal
        a = goal_p - curr_p
        rot_goal = np.arctan2(a[1], a[0])
        robot_angle = np.radians(self.robot_pose2d[robot][2])
        angle_diff = self.normalize_angle(rot_goal - robot_angle)
        
        cmd_vel = Twist()
        
        Kp_angular = 1.5  # joue this parameter
        cmd_vel.angular.z = Kp_angular * angle_diff
        
        max_angular_vel = 1.5  # Adjust as needed
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, max_angular_vel), -max_angular_vel)
        
        if abs(angle_diff) < np.radians(20):  # 20 degrees threshold
            distance = np.linalg.norm(a)
            Kp_linear = 0.2  # Tune this parameter
            cmd_vel.linear.x = Kp_linear * distance
            
            max_linear_vel = 0.2  # Adjust as needed
            cmd_vel.linear.x = min(cmd_vel.linear.x, max_linear_vel)
        
        # check proximity to other robots and adjust speed
        closest_distance = self.get_closest_robot_distance(robot)
        if closest_distance < 2:  # 10 cm threshold
            slowdown_factor = closest_distance * 0.1  # linear slowdown
            cmd_vel.linear.x *= slowdown_factor
            cmd_vel.angular.z *= slowdown_factor
        
        return cmd_vel

    def get_closest_robot_distance(self, robot):
        curr_pos = np.array(self.robot_pose2d[robot][:2])
        distances = []
        for other_robot, pose in self.robot_pose2d.items():
            if other_robot != robot:
                other_pos = np.array(pose[:2])
                distance = np.linalg.norm(curr_pos - other_pos)
                distances.append(distance)
        return min(distances) if distances else float('inf')

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def tb_path_receive(self, msg):
        tb_id = msg.tb_id
        data = msg.listofpath.data
        dim0_size = msg.listofpath.layout.dim[0].size
        dim1_size = msg.listofpath.layout.dim[1].size
        re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
        robot_id = f'robot{tb_id}'
        self.robot_path[robot_id] = re_tb_path
        
        # Convert path to timesteps
        self.robot_timesteps[robot_id] = self.path_to_timesteps(re_tb_path)
        self.max_timesteps = max(self.max_timesteps, len(self.robot_timesteps[robot_id]))
        
        # Reset movement counters
        self.robot_movements[robot_id]['straight'] = 0
        self.robot_movements[robot_id]['rotate'] = 0
        
        # Calculate straight movements and rotations
        for i in range(1, len(re_tb_path)):
            prev_point = re_tb_path[i-1]
            curr_point = re_tb_path[i]
            
            # Check if it's a straight movement
            if (prev_point[0] == curr_point[0] and prev_point[1] != curr_point[1]) or \
            (prev_point[1] == curr_point[1] and prev_point[0] != curr_point[0]):
                self.robot_movements[robot_id]['straight'] += 1
            
            # Check if it's a rotation (change in direction)
            if i > 1:
                prev_prev_point = re_tb_path[i-2]
                prev_direction = (prev_point[0] - prev_prev_point[0], prev_point[1] - prev_prev_point[1])
                curr_direction = (curr_point[0] - prev_point[0], curr_point[1] - prev_point[1])
                if prev_direction != curr_direction:
                    self.robot_movements[robot_id]['rotate'] += 1
        
        # Set start time when path is received
        if self.robot_time[robot_id]['start'] is None:
            self.robot_time[robot_id]['start'] = time.time()
        
        print(f"Robot {robot_id} movements - Straight: {self.robot_movements[robot_id]['straight']}, "
            f"Rotate: {self.robot_movements[robot_id]['rotate']}")

    def path_to_timesteps(self, path):
        timesteps = []
        for point in path:
            timesteps.append(point[:2])  # Only take x and y coordinates
        return timesteps

    def log_robot_time(self, robot):
        os.makedirs(f'{WS_DIR}/result', exist_ok=True)
        with open(f'{WS_DIR}/result/pathfollowing.txt', 'a') as f:
            f.write(f"{'='*50}\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Robot: {robot}\n")
            f.write(f"Execution time: {self.robot_time[robot]['duration']:.4f} seconds\n")
            f.write(f"Straight movements: {self.robot_movements[robot]['straight']}\n")
            f.write(f"Rotations: {self.robot_movements[robot]['rotate']}\n")
            f.write(f"Total path length: {self.robot_movements[robot]['straight'] + self.robot_movements[robot]['rotate']}\n")
            f.write(f"Average speed(Straight): {(self.robot_movements[robot]['straight']) / self.robot_time[robot]['duration']:.4f} straight/second\n")
            f.write(f"Average speed(m): {(self.robot_movements[robot]['straight']*3) / self.robot_time[robot]['duration']:.4f} m/second\n")
            f.write(f"{'='*50}\n\n")
            
    def calculate_total_movements(self):
        total_straight = sum(robot['straight'] for robot in self.robot_movements.values())
        total_rotate = sum(robot['rotate'] for robot in self.robot_movements.values())
        return total_straight, total_rotate

    def log_total_time(self):
        for robot in self.robots:
            if self.robot_time[robot]['end'] is None:
                self.robot_time[robot]['end'] = time.time()
                self.robot_time[robot]['duration'] = self.robot_time[robot]['end'] - self.robot_time[robot]['start']

        completed_durations = [robot['duration'] for robot in self.robot_time.values() if robot['duration'] is not None]
        duration_sum = sum(completed_durations)
        print(f"sum_durations: {duration_sum}")

        if not completed_durations:
            print("No robots have completed their paths yet.")
            return

        total_time = max(completed_durations)
        total_straight, total_rotate = self.calculate_total_movements()
        
        os.makedirs(f'{WS_DIR}/result', exist_ok=True)
        with open(f'{WS_DIR}/result/pathfollowing.txt', 'a') as f:
            f.write(f"{'#'*50}\n")
            f.write(f"OVERALL MISSION SUMMARY\n")
            f.write(f"{'#'*50}\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f'Total robot execution time: {duration_sum:.4f} seconds\n')
            f.write(f"Total straight movements(3m): {total_straight} , {total_straight*3} meter\n")
            f.write(f"Total rotations: {total_rotate}\n")
            f.write(f"Total path length: {total_straight + total_rotate}\n")
            f.write(f"average spd(straight): {(total_straight)/duration_sum:.4f} straight/sec , {(total_straight*3)/duration_sum:.4f} m/sec\n\n")
            f.write(f"Overall efficiency: {(total_straight + total_rotate) / duration_sum:.4f} units/second\n\n")
            
            f.write("Individual Robot Performance:\n")
            for robot, time_data in self.robot_time.items():
                if time_data['duration'] is not None:
                    f.write(f"  {robot}:\n")
                    f.write(f"    Time: {time_data['duration']:.4f} seconds\n")
                    f.write(f"    Straight: {self.robot_movements[robot]['straight']}\n")
                    f.write(f"    Rotate: {self.robot_movements[robot]['rotate']}\n")
                    f.write(f"    average spd(straight): {(self.robot_movements[robot]['straight']) / time_data['duration']:.4f} straight/sec\n")
                    f.write(f"    average spd(m): {(self.robot_movements[robot]['straight']*3) / time_data['duration']:.4f} m/sec\n")
                    f.write(f"    Efficiency: {(self.robot_movements[robot]['straight'] + self.robot_movements[robot]['rotate']) / time_data['duration']:.4f} units/second\n")
            
            f.write(f"{'#'*50}\n\n")
        
        print(f"All robots have completed their paths. Total execution time: {total_time:.4f} seconds")
        print(f"Total movements - Straight: {total_straight}, Rotate: {total_rotate}")

def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollowing()
    print('Path following node running')
    try:
        rclpy.spin(path_follower)
    except KeyboardInterrupt:
        print('Path following node terminated')
    finally:
        path_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray, Int32MultiArray
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
        super().__init__('Path_Follower')
        
        # self.robots = [f'robot{i+1}' for i in range(TOTAL_ROBOTS)]
        self.robots = [f'robot{i}'for i in range(1,TOTAL_ROBOTS+1)]
        # self.robots = ['robot1', 'robot2', 'robot3']
        self.robot_pose2d = {robot: [0, 0, 0] for robot in self.robots}
        self.robot_path = {robot: None for robot in self.robots}
        self.robot_goals_reached = {robot: False for robot in self.robots}
        self.robot_cur_goal = {robot: None for robot in self.robots}
        self.robot_time = {robot: {'start': None, 'end': None, 'duration': None} for robot in self.robots}
        self.robot_movements = {robot: {'straight': 0, 'rotate': 0} for robot in self.robots}
        self.robot_head_to = {robot: None for robot in self.robots}
        # self.reached_goal_timestep = [{**{robot: False for robot in self.robots}, 'timestep': j} for j in range(max(len(i) for i in self.robot_path))]
        # self.robot_timestep = None

        # to make stop time        
        self.robot_wait_time = {robot: 0 for robot in self.robots}
        self.robot_wait_start = {robot: None for robot in self.robots}

        
        self.all_robots_completed = False
        
        self.all_robots_exec_time = []
        
        # Subscriptions
        self.create_subscription(Odometry, 'tb_1_green/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_2_red/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_3_blue/odom', self.tb_odom_cb, 10)
        self.create_subscription(Odometry, 'tb_4_sky/odom', self.tb_odom_cb, 10)
        
        self.create_subscription(Tbpose2D, 'tb_pose2d', self.sub_tb2d_pos, 10)
        self.create_subscription(TbPathtoGoal, "TbPathtoGoal_top", self.tb_path_receive, 10)        
        self.create_subscription(TbPathtoGoal, "Tb_head_to", self.tb_head_to_receive, 10)

        
        # Publishers
        self.pub = {robot: self.create_publisher(Twist, f'/{robot}/cmd_vel', 10) for robot in self.robots}
        
        self.robot_is_moving = {robot: 0 for robot in self.robots}
        # New publisher for robot movement status
        self.movement_status_pub = self.create_publisher(Int32MultiArray, '/robot_is_moving', 10)
        
        
        # Timer to publish cmd_vel periodically
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10 Hz update rate

    def tb_odom_cb(self, msg):
        pass  

    def sub_tb2d_pos(self, msg):
        robot_id = f'robot{msg.tb_id}'
        self.robot_pose2d[robot_id] = [msg.tb_pos.x, msg.tb_pos.y, msg.tb_pos.theta]
    
    def publish_cmd_vel(self):
        all_completed = True
        movement_status = Int32MultiArray()
        movement_status.data = [self.robot_is_moving[robot] for robot in self.robots]

        for robot in self.robots:
            if self.robot_path[robot] is not None and len(self.robot_path[robot]) > 0:
                all_completed = False
                if self.robot_goals_reached[robot]:
                    self.robot_goals_reached[robot] = False
                    self.robot_path[robot].pop(0)

                if len(self.robot_path[robot]) == 0:
                    self.robot_path[robot] = None
                    self.robot_time[robot]['end'] = time.time()
                    self.robot_time[robot]['duration'] = self.robot_time[robot]['end'] - self.robot_time[robot]['start']
                    self.robot_is_moving[robot] = 0
                    continue

                self.robot_cur_goal[robot] = np.array(self.robot_path[robot][0][:2])
                self.robot_is_moving[robot] = 1
                
                # cmd_vel = self.calculate_cmd_vel(robot)
                # self.pub[robot].publish(cmd_vel)

            cmd_vel = self.calculate_cmd_vel(robot)
            self.pub[robot].publish(cmd_vel)

        # Update movement status data
        movement_status.data = [self.robot_is_moving[robot] for robot in self.robots]
        
        # Publish movement status
        self.movement_status_pub.publish(movement_status)

        if all_completed and not self.all_robots_completed:
            if all(robot['duration'] is not None for robot in self.robot_time.values()):
                self.all_robots_completed = True
                self.log_total_time()

    def calculate_cmd_vel(self, robot):
        if self.robot_cur_goal[robot] is None or self.robot_path[robot] is None or len(self.robot_path[robot]) == 0:
            return self.stop()
        
        current_goal = np.array(self.robot_path[robot][0][:2])
        curr_p = np.array(self.robot_pose2d[robot][:2])
        
        # check if reached the current goal
        if np.linalg.norm(curr_p - current_goal) < 0.4:  # 20cm 
            # Find the next non-repeat goal
            next_non_repeat_index = 1
            while next_non_repeat_index < len(self.robot_path[robot]) and np.array_equal(self.robot_path[robot][next_non_repeat_index][:2], current_goal):
                next_non_repeat_index += 1
            
            # repeats = next_non_repeat_index - 1
            repeats = next_non_repeat_index

            
            if repeats > 1:
                if self.robot_wait_start[robot] is None:
                    self.robot_wait_start[robot] = time.time()
                
                # # Wait for the first second
                # if time.time() - self.robot_wait_start[robot] < 1.0:
                #     return self.stop()
                
                if next_non_repeat_index < len(self.robot_path[robot]):
                        next_goal = np.array(self.robot_path[robot][next_non_repeat_index][:2])
                        a = next_goal - curr_p
                        rot_goal = np.arctan2(a[1], a[0])
                        robot_angle = np.radians(self.robot_pose2d[robot][2])
                        angle_diff = self.normalize_angle(rot_goal - robot_angle)
                        
                        if abs(angle_diff) > np.radians(5):  # If we need to rotate more than 5 degrees
                            if angle_diff > 0:
                                return self.rotate_left()
                            else:
                                return self.rotate_right()
                        # else: 
                        #     # if time.time() - self.robot_wait_start[robot] < 1.0:
                        #     return self.stop()
                    
                if time.time() - self.robot_wait_start[robot] < repeats - 0.07:
                    return self.stop()
                
                # If there's more than one repeat, use the second repeat for rotation
                # if repeats > 1:
                #     if next_non_repeat_index < len(self.robot_path[robot]):
                #         next_goal = np.array(self.robot_path[robot][next_non_repeat_index][:2])
                #         a = next_goal - curr_p
                #         rot_goal = np.arctan2(a[1], a[0])
                #         robot_angle = np.radians(self.robot_pose2d[robot][2])
                #         angle_diff = self.normalize_angle(rot_goal - robot_angle)
                        
                #         if abs(angle_diff) > np.radians(10):  # If we need to rotate more than 5 degrees
                #             if angle_diff > 0:
                #                 return self.rotate_left()
                #             else:
                #                 return self.rotate_right()
                    
                #     # If rotation is complete or not needed, wait for the remaining time
                #     if time.time() - self.robot_wait_start[robot] < repeats:
                #         return self.stop()
            
            self.robot_wait_start[robot] = None
            self.robot_goals_reached[robot] = True
            for _ in range(next_non_repeat_index - 1): 
                self.robot_path[robot].pop(0)
            return self.stop()

        a = current_goal - curr_p
        rot_goal = np.arctan2(a[1], a[0])
        robot_angle = np.radians(self.robot_pose2d[robot][2])
        angle_diff = self.normalize_angle(rot_goal - robot_angle)
        
        if abs(angle_diff) < np.radians(10):  # faacing
            return self.go_straight()
        elif angle_diff > 0:
            cmd_vel =  self.rotate_left()
        else:
            cmd_vel = self.rotate_right()
        
        # closest_distance = self.get_closest_robot_distance(robot)
        # if closest_distance < 3:  # 10 cm threshold
        #     slowdown_factor = closest_distance * 0.1  # linear slowdown
        #     cmd_vel.linear.x *= slowdown_factor
        #     cmd_vel.angular.z *= slowdown_factor
        
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

    def check_wait_time(self, robot):
        if len(self.robot_path[robot]) < 2:
            return 0
        
        current_goal = self.robot_path[robot][0][:2]
        wait_time = 0
        
        for next_goal in self.robot_path[robot][1:]:
            if np.array_equal(current_goal, next_goal[:2]):
                wait_time += 1
            else:
                break
        
        return wait_time

    def go_straight(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.67
        return cmd_vel

    def rotate_left(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.67 
        return cmd_vel

    def rotate_right(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = -0.67  
        return cmd_vel

    def stop(self):
        return Twist()  

    def backward(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.6 
        return cmd_vel
    
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def tb_head_to_receive(self, msg):
        print(f'Received head_to message')
        tb_id = msg.tb_id
        data = msg.listofpath.data
        dim0_size = msg.listofpath.layout.dim[0].size
        dim1_size = msg.listofpath.layout.dim[1].size
        re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
        # print(f'head_to_re: {re_tb_path}')
        robot_id = f'robot{tb_id}'
        self.robot_head_to[robot_id] = re_tb_path if re_tb_path else []  # Set to empty list if re_tb_path is falsy
        # print(f'head_to : ')
        # for i, j in self.robot_head_to.items():
        #     print(f'{i}')
            # if j:  # Only iterate if j is not None and not empty
            #     for k in j: 
            #         print(k)
        # print(f'path_to : ')
        # for i, j in self.robot_path.items():
        #     print(f'{i}, {j}')

    
    def tb_path_receive(self, msg):
        tb_id = msg.tb_id
        data = msg.listofpath.data
        dim0_size = msg.listofpath.layout.dim[0].size
        dim1_size = msg.listofpath.layout.dim[1].size
        re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
        
        robot_id = f'robot{tb_id}'
        self.robot_path[robot_id] = re_tb_path if re_tb_path else []
        print(f'pathttt : ')
        for i, j in self.robot_path.items():
            print(f'{i}')
            if j:  # Only iterate if j is not None and not empty
                for k in j: 
                    print(k)
        # print(f'path_to : ')
        # for i, j in self.robot_path.items():
        #     print(f'{i}, {j}')
        
        # Set robot as moving when it receives a new path
        self.robot_is_moving[robot_id] = 1
        
        # Set start time when path is received
        if self.robot_time[robot_id]['start'] is None:
            self.robot_time[robot_id]['start'] = time.time()
        
        # Reset movement counters
        self.robot_movements[robot_id]['straight'] = 0
        self.robot_movements[robot_id]['rotate'] = 0
        
        # Calculate straight movements and rotations
        if re_tb_path and len(re_tb_path) > 1:
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
                    
        print(f"Robot {robot_id} movements - Straight: {self.robot_movements[robot_id]['straight']}, "
            f"Rotate: {self.robot_movements[robot_id]['rotate']}")

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
        completed_durations = [robot['duration'] for robot in self.robot_time.values() if robot['duration'] is not None]
        durtation_sum = sum([robot['duration'] for robot in self.robot_time.values() if robot['duration'] is not None])
        print(f"sum_durations: {durtation_sum}")
        if not completed_durations:
            print("No robots have completed their paths yet.")
            return

        total_time = max(completed_durations)
        total_straight, total_rotate = self.calculate_total_movements()
        
        # Ensure the directory exists
        os.makedirs(f'{WS_DIR}/result', exist_ok=True)
        with open(f'{WS_DIR}/result/pathfollowing.txt', 'a') as f:
            f.write(f"{'#'*50}\n")
            f.write(f"OVERALL MISSION SUMMARY\n")
            f.write(f"{'#'*50}\n")
            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            # f.write(f"Total execution time: {total_time:.4f} seconds\n")
            f.write(f'Total robot execution time: {durtation_sum:.4f} seconds\n')
            f.write(f"Total straight movements(3m): {total_straight} , {total_straight*3} meter\n") # 3m per straight movement
            f.write(f"Total rotations: {total_rotate}\n")
            f.write(f"Total path length: {total_straight + total_rotate}\n")
            f.write(f"average spd(straight): {(total_straight)/durtation_sum:.4f} straight/sec , {(total_straight*3)/durtation_sum:.4f} m/sec\n\n")
            f.write(f"Overall efficiency: {(total_straight + total_rotate) / durtation_sum:.4f} units/second\n\n")
            
            f.write("Individual Robot Performance:\n")
            for robot, time_data in self.robot_time.items():
                if time_data['duration'] is not None:
                    f.write(f"  {robot}:\n")
                    f.write(f"    Time: {time_data['duration']:.4f} seconds\n")
                    f.write(f"    Straight: {self.robot_movements[robot]['straight']}\n")
                    f.write(f"    Rotate: {self.robot_movements[robot]['rotate']}\n")
                    f.write(f"    average spd(straight): {(self.robot_movements[robot]['straight']) / self.robot_time[robot]['duration']:.4f} straight/sec\n")
                    f.write(f"    average spd(m): {(self.robot_movements[robot]['straight']*3) / self.robot_time[robot]['duration']:.4f} m/sec\n")
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
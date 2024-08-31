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
from all_planner.DICISION import CONV_NORMAL_TO_POS,CONV_POS_TO_NORMAL,WS_DIR,MAP_NAME,TOTAL_ROBOTS,OBSTACLES_MAP
from all_planner.ex_planner_control import PlannerControl
import glob
import sys
from all_planner.Robot_State_Check_Extension import Robot_State_Check


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

        self.robot_goal_only = {robot: None for robot in self.robots}
        
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
        
        self.pub_path_goal = self.create_publisher(TbPathtoGoal, 'TbPathtoGoal_top', 10)
        # Timer to publish cmd_vel periodically
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # 10 Hz update rate

        self.map = glob_map_files()
        if self.map is None:
            self.get_logger().error('No map file found. Exiting.')
            rclpy.shutdown()
            sys.exit(1)
    def tb_odom_cb(self, msg):
        pass  

    def sub_tb2d_pos(self, msg):
        robot_id = f'robot{msg.tb_id}'
        self.robot_pose2d[robot_id] = [msg.tb_pos.x, msg.tb_pos.y, msg.tb_pos.theta]
    
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
    
    def publish_cmd_vel(self):
        all_completed = True
        movement_status = Int32MultiArray()
        movement_status.data = [self.robot_is_moving[robot] for robot in self.robots]

        for robot in self.robots:
            if self.robot_path[robot] is not None and len(self.robot_path[robot]) > 0:
                all_completed = False
                if self.robot_goals_reached[robot]:
                    self.robot_goals_reached[robot] = False
                    # self.robot_path[robot].pop(0)

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
        current_angle_goal = self.robot_path[robot][0][-1]
        curr_p = np.array(self.robot_pose2d[robot][:2])
        
        # check if reached the current goal
        if np.linalg.norm(curr_p - current_goal) < 0.4:  # 20cm 
            robot_angle = np.radians(self.robot_pose2d[robot][2])
            angle_goal_diff = self.normalize_angle(np.radians(current_angle_goal) - robot_angle)
            
            if abs(angle_goal_diff) < np.radians(10):  # close to the desired final angle
                # Check if the next waypoint (if it exists) is the same as the current one
                if len(self.robot_path[robot]) > 1 and np.array_equal(self.robot_path[robot][0], self.robot_path[robot][1]):
                    if not hasattr(self, 'wait_start_time'):
                        self.wait_start_time = time.time()
                    
                    if time.time() - self.wait_start_time >= 0.00:  # Wait for 1 second
                        self.robot_goals_reached[robot] = True
                        self.robot_path[robot].pop(0)
                        delattr(self, 'wait_start_time')
                        
                        # if len(self.robot_path[robot]) > 1 and self.robot_path[robot] is not None and self.robot_goals_reached[robot] == True:
                        #     # for robot
                        #     starts = []
                        #     initial_angles = []
                        #     for robot in self.robots:
                        #         start = []
                        #         if self.robot_path[robot] and len(self.robot_path[robot]) > 0:
                        #             start.extend(self.robot_path[robot][0][:2])
                        #             initial_angles.append(self.robot_path[robot][0][-1])
                        #         else:
                        #             initial_angles.append(self.robot_goal_only[robot][-1])
                        #         starts.append(start if start else self.robot_goal_only[robot][:2])  # Append None if start is empty
                                
                        #     goals = [(self.robot_goal_only[robot][:2]) for robot in self.robots]
                        #     normal_start = [CONV_POS_TO_NORMAL(starts[i]) for i in range(len(starts))]
                        #     normal_goals = [CONV_POS_TO_NORMAL(goals[i]) for i in range(len(goals))]
                        #     print(f'starts: \n{starts}')
                        #     print(f'goal: \n{goals}')
                        #     print(f'starts_n: \n{normal_start}')
                        #     print(f'goal_n: \n{normal_goals}')
                        #     print(f'initial_angle: \n{initial_angles}')
                        #     print('IIIIIIIIIIIIIIIII')
                        #     solution = PlannerControl().plan_paths_with_rot_state(self.map, normal_start, normal_goals,initial_angles)
                        #     paths, nodes_gen, nodes_exp , head_to = solution[:4]
                            
                        #     all_paths_conv = [np.array([CONV_NORMAL_TO_POS(point) for point in path]) for path in paths] 
                        #     print(f'all paths conv w/ to ROBOT state ')
                        #     robot_state = []
                        #     for i, arr in enumerate(all_paths_conv):
                        #         # convert the values to add to a numpy array with the correct shape
                        #         add_values = np.array(head_to[i]).reshape(arr.shape[0], 1)
                                
                        #         # concatenate the original array with the new values
                        #         new_arr = np.concatenate((arr, add_values), axis=1)
                                
                        #         robot_state.append(new_arr)
                            
                        #     for i in range(len(all_paths_conv)):
                        #         # self.robot_path[f'robot{i+1}'] = all_paths_conv[i].tolist()
                        #         if robot == f'robot{i+1}': 
                        #             self.robot_path[f'robot{i+1}'] = robot_state[i]
                        
                        
                    
                    return self.stop()
                else:
                    self.robot_goals_reached[robot] = True
                    self.robot_path[robot].pop(0)
                    
                    #replan
                    # print(self.robot_path)
                    
                    # if len(self.robot_path[robot]) > 0 and self.robot_path[robot] is not None and self.robot_goals_reached[robot] == True:
                    #     # for robot
                    #     starts = []
                    #     initial_angles = []
                    #     for robot in self.robots:
                    #         start = []
                    #         if len(self.robot_path[robot]) > 0:
                    #             start.extend(self.robot_path[robot][0][:2])
                    #             initial_angles.append(self.robot_path[robot][0][-1])
                    #         else:
                    #             initial_angles.append(self.robot_goal_only[robot][-1])
                    #         starts.append(start if start else self.robot_goal_only[robot][:2])  # Append None if start is empty
                            
                    #     goals = [(self.robot_goal_only[robot][:2]) for robot in self.robots]
                    #     normal_start = [CONV_POS_TO_NORMAL(starts[i]) for i in range(len(starts))]
                    #     normal_goals = [CONV_POS_TO_NORMAL(goals[i]) for i in range(len(goals))]
                    #     print(f'starts: \n{starts}')
                    #     print(f'goal: \n{goals}')
                    #     print(f'starts_n: \n{normal_start}')
                    #     print(f'goal_n: \n{normal_goals}')
                    #     print(f'initial_angle: \n{initial_angles}')
                    #     print('IIIIIIIIIIIIIIIII')
                    #     solution = PlannerControl().plan_paths_with_rot_state(self.map, normal_start, normal_goals,initial_angles)
                    #     paths, nodes_gen, nodes_exp , head_to = solution[:4]
                        
                    #     rotation_checker = Robot_State_Check()  # for test
                        
                        
                    #     robot_exten_paths = []
                    #     for i in range(len(paths)):
                    #         print(f'all_path_i :{paths[i]}')
                    #         print(f'head_to_i :{head_to[i]}')
                    #         robot_exten_paths.append(rotation_checker.combine_robot_state_list(head_to[i],paths[i]))
                            
                    #     angle_correct = []
                    #     for i in robot_exten_paths: angle_correct.append(rotation_checker.make_angle_correct(i))
                    #     # result = rotation_checker.combine_robot_state_list(angle, pos)
                    #     print(f'ANGLEEE: \n{angle_correct}')
                    #     # result2 = rotation_checker.detect_add_state(result)
                    #     # print(f'RESULTTT@222: \n{result2}')
                    #     robot_exten_pos2 = []
                    #     for i in angle_correct:
                    #         robot_exten_pos2.append(rotation_checker.get_position_state(i))
                    #     print(f'POS AFTER: \n{robot_exten_pos2}')
                    #     robot_exten_angle2 = []
                    #     for i in angle_correct:
                    #         robot_exten_angle2.append(rotation_checker.get_angle_state(i))
                    #     print(f'ANGLE AFTER: \n{robot_exten_angle2}')
                        
                    #     all_paths_conv2 = [np.array([CONV_NORMAL_TO_POS(point) for point in path]) for path in robot_exten_pos2]
                        
                    #     robot_state3 = []
                    #     for i, arr in enumerate(all_paths_conv2):
                    #         # convert the values to add to a numpy array with the correct shape
                    #         add_values = np.array(robot_exten_angle2[i]).reshape(arr.shape[0], 1)
                            
                    #         # concatenate the original array with the new values
                    #         new_arr = np.concatenate((arr, add_values), axis=1)
                            
                    #         robot_state3.append(new_arr)
                        
                        # all_paths_conv = [np.array([CONV_NORMAL_TO_POS(point) for point in path]) for path in paths] 
                        # print(f'all paths conv w/ to ROBOT state ')
                        # robot_state = []
                        # for i, arr in enumerate(all_paths_conv):
                        #     # convert the values to add to a numpy array with the correct shape
                        #     add_values = np.array(head_to[i]).reshape(arr.shape[0], 1)
                            
                        #     # concatenate the original array with the new values
                        #     new_arr = np.concatenate((arr, add_values), axis=1)
                            
                        #     robot_state.append(new_arr)
                        
                        # for i in range(len(robot_state3)):
                        #     if robot == f'robot{i+1}': 
                        #         self.robot_path[f'robot{i+1}'] = robot_state3[i].tolist()
                                # self.robot_path[f'robot{i+1}'] = robot_state[i]
                        # sorted_tb_id = [1, 2, 3, 4]
                        # self.pub_tb_path(sorted_tb_id, robot_state3)
                        
                    return self.stop()
            else:
                # We're at the right position but need to rotate to the final angle
                if angle_goal_diff > 0:
                    return self.rotate_left()
                else:
                    return self.rotate_right()

        a = current_goal - curr_p
        rot_goal = np.arctan2(a[1], a[0])
        robot_angle = np.radians(self.robot_pose2d[robot][2])
        angle_diff = self.normalize_angle(rot_goal - robot_angle)
        
        if abs(angle_diff) < np.radians(15):  # facing the right direction for movement
            return self.go_straight()
        elif angle_diff > 0:
            return self.rotate_left()
        else:
            return self.rotate_right()

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
        print(f'path_to : ')
        for i, j in self.robot_path.items():
            print(f'{i}, {j}')
        
        # Set robot as moving when it receives a new path
        self.robot_is_moving[robot_id] = 1
        
        #get robot goal only
        self.robot_goal_only[robot_id] = self.robot_path[robot_id][-1]
        print(f'goal_only: {self.robot_goal_only}')
        
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
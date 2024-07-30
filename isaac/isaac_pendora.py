# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
# simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.objects import VisualCone, VisualSphere, VisualCylinder, VisualCuboid
import numpy as np
import signal
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import omni.isaac.core.utils.stage as stage_utils
from omni.isaac.core.utils.rotations import euler_angles_to_quat ,quat_to_euler_angles
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.articulations import Articulation
from omni.isaac.cloner import GridCloner,Cloner
from omni.isaac.core.prims import XFormPrimView
from omni.isaac.core.utils.extensions import enable_extension
import time
import omni.isaac.core.utils.prims as prim_utils

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D
from mapf_isaac.msg import TbPathtoGoal,TbTask,Tbpose2D
from all_planner.DICISION import ROBOT_START,TOTAL_ROBOTS

def create_link_graph(a_y,a_x,b_y,b_x):
    pos_rot = {}
    # print(f'lower-> a_x:{b_x} ,b_y:{b_y}')
    # print(f'upper-> a_x:{a_x} ,a_y:{a_y}')
    pos_link_x = a_x + b_x
    pos_link_y = a_y + b_y
    pos_node_link = np.array([pos_link_x/2,pos_link_y/2,0.25])
    rota_link_x = 90 if a_x == b_x else 0
    rota_link_y = 90 if a_y == b_y else 0    
    rota_node_link = np.array([rota_link_x,rota_link_y,0.0])
    pos_rot = {
        "pos": pos_node_link,
        "rot": euler_angles_to_quat(rota_node_link,degrees=True,extrinsic=False),
    }
    return pos_rot
         
class Subscriber(Node):
    def __init__(self):
        super().__init__("Simulation_Isaac_UI")
        # setting up the world with a cube
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)
        # self.ros_world.scene.add_default_ground_plane()
        
        self.all_paths_count = 0
        
        warehouse_path = "/home/eggs/obo_isaac_simulation/usd/pendora4.usd"
        warehouse_prim_path = "/World/pendora"
        stage_utils.add_reference_to_stage(usd_path=warehouse_path, prim_path=warehouse_prim_path)
        warehouse = Articulation(prim_path=warehouse_prim_path, name="warehousee",position=np.array([-0.05407,10.37693,0.0]))
        tb_usd_path_green = "/home/eggs/obo_isaac_simulation/usd/turtlebot3green_new.usd"
        tb_usd_path_red = "/home/eggs/obo_isaac_simulation/usd/turtlebot3red.usd"
        tb_usd_path_blue = "/home/eggs/obo_isaac_simulation/usd/turtlebot3blue.usd"
        tb_usd_path_sky = "/home/eggs/obo_isaac_simulation/usd/turtlebot4sky.usd"
        tb_yellow = "/home/eggs/obo_isaac_simulation/usd/turtlebot3yellow.usd"
        tb_purple = "/home/eggs/obo_isaac_simulation/usd/turtlebot3purple.usd"
        tb_white = "/home/eggs/obo_isaac_simulation/usd/turtlebot3white.usd"
        tb_black = "/home/eggs/obo_isaac_simulation/usd/turtlebot3black.usd"
        
        cylider_light_1 = prim_utils.create_prim(
            "/World/CylinderLight_1",
            "CylinderLight",
            position=np.array([0.0, 0.0, 67.0]),
            attributes={
                "inputs:radius": 5.0,
                "inputs:intensity": 30000,
                "inputs:color": (1.0, 1.0, 1.0),
                "inputs:length": 100.0,
            }
        )
        
        #-------------------- declare param

        # c =        green , red   , blue  , bluesky,yellow, purple, white , black
        tb_color = [[0,1,0],[1,0,0],[0,0,1],[0,1,1],[1,1,0],[1,0,1],[1,1,1],[0,0,0]]
        tb_usd_path = [tb_usd_path_green, tb_usd_path_red, tb_usd_path_blue,tb_usd_path_sky,tb_yellow,tb_purple,tb_white,tb_black]
        start_pos = [j for i,j in ROBOT_START.items()]
        # print('start_pos :',start_pos)
        
        self.tb_job = [
            {
                "id" : i,
                "name": f'tb_{i}',
                "color": np.array(tb_color[i-1]),
                "start_pos": np.array(start_pos[i-1]),
                "usd_path": tb_usd_path[i-1]
            } for i in range(1,TOTAL_ROBOTS+1)
        ]
        print(f'len_tb_job: {len(self.tb_job)}')
        for i in range(len(self.tb_job)):
            self.ros_world.scene.add(
                WheeledRobot(
                    prim_path=f'/World/Tb_{i+1}',
                    name=self.tb_job[i]["name"],
                    create_robot=True,
                    wheel_dof_names=["wheel_left_joint", "wheel_right_joint"],
                    usd_path=self.tb_job[i]["usd_path"],
                    position=self.tb_job[i]["start_pos"],
                    orientation=np.array(euler_angles_to_quat([0,0,90],degrees=True,extrinsic=False))        
                )
            )
        clonerr = GridCloner(spacing=3,num_per_row=54)
        # cloner = GridCloner(spacing=3,num_per_row=18)
        sph_node_prim_path = "/World/node"
        VisualSphere(
                prim_path=sph_node_prim_path,
                name="sph_node_name",
                position=np.array([-23.5,-18.75,0.0]),
                scale=np.array([0.2,0.2,0.2]),
                color=np.array([1,1,1]),
                visible=True,
        )
        target_paths_sph = clonerr.generate_paths("/World/node",88)
        print(f't:{len(target_paths_sph)}')
        num_sph_nodes = len(target_paths_sph)
        clonerr.clone(source_prim_path="/World/node", prim_paths=target_paths_sph)
        
        # cloner more for left node
        cloner = Cloner()
        target_paths = cloner.generate_paths("/World/node_", 32)
        sph_left_pos = np.array([[4.5 if k==0 else -4.5,-61.5+(i*15 if i<2 else i*15+3*(i-1))+(j*3),0] for i in range(8) for j in range(2) for k in range(2)])
        cloner.clone(source_prim_path="/World/node", prim_paths=target_paths, positions=sph_left_pos)
        
        # obstacle 
        # target_paths_ob = cloner.generate_paths("/World/node_o_", 72)
        # target_paths_ob2 = cloner.generate_paths("/World/node_o2_", 12)
        # target_paths_ob3 = cloner.generate_paths("/World/node_o3_", 4)
        # target_paths_ob4 = cloner.generate_paths("/World/node_o4_", 4)
        # target_paths_ob5 = cloner.generate_paths("/World/node_o5_", 2)
        target_paths_cbo = cloner.generate_paths("/World/node_cbo_", 94)
        
        
        ob = np.array([[7.5 if j==0 and k<=1 else -7.5 if j==1 and k<=1 else 4.5 if j==0 else -4.5  ,-46.5+(i*18)+(k*3),0] for i in range(6) for j in range(2) for k in range(6)])
        ob_2 = np.array([[7.5 if (i==1 and j==0) or (i==2 and j==0) else -7.5 if (i==1 and j==1) or (i==2 and j==1) else 4.5 if j==0 else -4.5,-64.5+(i*3),0] for i in range(6) for j in range(2)])
        ob_3 = np.array([[4.5-(i*3),67.5,0] for i in range(4)])
        ob_4 = np.array([[7.5 if j==0 else -7.5,61.5+(i*3),0] for i in range(2) for j in range(2)])
        ob_5 = np.array([[1.5,-67.5,0],[-1.5,-67.5,0]])
        ob_combined = np.concatenate((ob,ob_2,ob_3,ob_4, ob_5), axis=0)
        len(ob_combined)
        # cloner.clone(source_prim_path="/World/node", prim_paths=target_paths_ob, positions=ob)
        # cloner.clone(source_prim_path="/World/node", prim_paths=target_paths_ob2, positions=ob_2)
        # cloner.clone(source_prim_path="/World/node", prim_paths=target_paths_ob3, positions=ob_3)
        # cloner.clone(source_prim_path="/World/node", prim_paths=target_paths_ob4, positions=ob_4)
        # cloner.clone(source_prim_path="/World/node", prim_paths=target_paths_ob5, positions=ob_5)
        # cloner.clone(source_prim_path="/World/node", prim_paths=target_paths_cbo, positions=ob_combined)
        
        
        sphs = XFormPrimView("/World/node_*")
        sph_nodes_positions , orien = sphs.get_world_poses()
        sph_nodes_positions[:,2] += 0.25
        sphs.set_world_poses(sph_nodes_positions, orien)
        for i in range(len(sph_nodes_positions)): print(f'{i}: {sph_nodes_positions[i]}')

        # setup the ROS2 subscriber here
        self.tb_pos = self.create_publisher(TbTask,'all_tb_pos',10)
        self.pose2d = self.create_publisher(Tbpose2D,'tb_pose2d',10)
        # self.initial_tb_pos = self.create_publisher(TbTask,'tb_initial_pos',10)
        self.ros_sub = self.create_subscription(TbPathtoGoal, "TbPathtoGoal_top", self.tb_path_receive, 10)
        self.ros_world.reset()
        self.tb_posi = TbTask()
        self.tb_pose = Tbpose2D()
        # for i in range(len(self.tb_job)):
        #     self.pub_tb_initial_pos(i+1,self.tb_job[i]["start_pos"])
            
    def pub_tb_pose2d(self,tb_id,pos,rot):
        self.tb_pose.tb_id = tb_id
        pose2d = Pose2D()
        pose2d.x = round(float(pos[0]),2)
        pose2d.y = round(float(pos[1]),2)
        euler = np.array(quat_to_euler_angles(rot,degrees=True, extrinsic=True))
        # print(f'rot : {euler}')
        # print(f'rot2 : {rot}')
        print(f'euler: {round(float(euler[2]),4)}')
        pose2d.theta = round(float(euler[2]),4)
        self.tb_pose.tb_pos = pose2d
        self.pose2d.publish(self.tb_pose)
        print(f'pose2d : {self.tb_pose}')
    
    # def tb_path_receive(self, msg):
    #     tb_id = msg.tb_id
    #     data = msg.listofpath.data
    #     dim0_size = msg.listofpath.layout.dim[0].size
    #     dim1_size = msg.listofpath.layout.dim[1].size
    #     re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
    #     print(f'len_tb_path: {len(re_tb_path)}')
    #     print(f'tb_path: {re_tb_path}')
    #     a = [i for i in range(1,len(re_tb_path))]
    #     b = [i for i in range(0,len(re_tb_path)-1)]
    #     for i in range(len(re_tb_path)-1):
    #         print(f'a_y: {re_tb_path[i+1][1]}, a_x: {re_tb_path[i+1][0]}, b_y: {re_tb_path[i][1]}, b_x: {re_tb_path[i][0]}')
    #         pos_rot = create_link_graph(re_tb_path[i+1][1], re_tb_path[i+1][0], re_tb_path[i][1], re_tb_path[i][0])
    #         print(f'pos_rot: {pos_rot}')
    #         self.ros_world.scene.add(
    #             VisualCylinder(
    #                 position=pos_rot["pos"],
    #                 prim_path=f'/World/Link_{tb_id}_{i+1}',
    #                 name=f'link_{tb_id}_{i+1}',
    #                 orientation=pos_rot["rot"],
    #                 scale=np.array([0.1,0.1,3.0]),
    #                 color=self.tb_job[tb_id-1]["color"]
    #             )
    #         )
        
    def tb_path_receive(self, msg):
        tb_id = msg.tb_id
        data = msg.listofpath.data
        dim0_size = msg.listofpath.layout.dim[0].size
        dim1_size = msg.listofpath.layout.dim[1].size
        re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
        print(f'len_tb_path: {len(re_tb_path)}')
        print(f'tb_path: {re_tb_path}')

        for i in range(len(re_tb_path) - 1):
            current_point = re_tb_path[i]
            next_point = re_tb_path[i + 1]
            
            print(f'a_y: {next_point[1]}, a_x: {next_point[0]}, b_y: {current_point[1]}, b_x: {current_point[0]}')
            
            # Check if the current point and next point are different
            if not (next_point[1] == current_point[1] and next_point[0] == current_point[0]):
                pos_rot = create_link_graph(next_point[1], next_point[0], current_point[1], current_point[0])
                print(f'pos_rot: {pos_rot}')
                
                self.ros_world.scene.add(
                    VisualCylinder(
                        position=pos_rot["pos"],
                        prim_path=f'/World/Link_{tb_id}_{i+1}_{self.all_paths_count}',
                        name=f'link_{tb_id}_{i+1}_{self.all_paths_count}',
                        orientation=pos_rot["rot"],
                        scale=np.array([0.1, 0.1, 3.0]),
                        color=self.tb_job[tb_id-1]["color"]
                    )
                )
                
                self.all_paths_count += 1
            else:
                print(f"Skipping VisualCylinder for identical consecutive points at index {i}")
    def run_simulation(self):
        self.timeline.play()
        reset_needed = False
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ros_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.ros_world.is_playing():
                if reset_needed:
                    self.ros_world.reset()
                    reset_needed = False
                # the actual setting the cube pose is done here
                for i in range(len(self.tb_job)):
                    pos, rot = self.ros_world.scene.get_object(self.tb_job[i]["name"]).get_world_pose()
                    self.pub_tb_pose2d(i+1,pos,rot)
                print()
                # self.ros_world.scene.get_object("cube_1").set_world_pose(self._cube_position)

        # Cleanup
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    subscriber = Subscriber()
    subscriber.run_simulation()

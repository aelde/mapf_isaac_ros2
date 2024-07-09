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
from omni.isaac.cloner import GridCloner
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
        
        warehouse_path = "/home/eggs/obo_isaac_simulation/usd/warehouse_nowall_nolamp.usd"
        warehouse_prim_path = "/World/warehouse"
        stage_utils.add_reference_to_stage(usd_path=warehouse_path, prim_path=warehouse_prim_path)
        warehouse = Articulation(prim_path=warehouse_prim_path, name="warehousee",position=np.array([10.139,-3.53467,0.0]))
        tb_usd_path_green = "/home/eggs/obo_isaac_simulation/usd/turtlebot3green_new.usd"
        tb_usd_path_red = "/home/eggs/obo_isaac_simulation/usd/turtlebot3red.usd"
        tb_usd_path_blue = "/home/eggs/obo_isaac_simulation/usd/turtlebot3blue.usd"
        
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
        self.tb_job = [
            {"id": 1, 
             "name": "tb_1",
             "color": np.array([0,1,0]), #green
             "start_pos": np.array([-1.5,-16.5,0.0]), 
             "usd_path": tb_usd_path_green
             }, 
            {"id": 2,
             "name": "tb_2",
             "color": np.array([1,0,0]), #red
             "start_pos" : np.array([7.5,19.5,0.0]),
             "usd_path": tb_usd_path_red
             }, 
            {"id": 3,
             "name": "tb_3",
             "color": np.array([0,0,1]), #blue
             "start_pos" : np.array([4.5,-16.5,0.0]),
            #  "start_pos" : np.array([1.5,-13.5,0.0]),
             "usd_path": tb_usd_path_blue
             }, 
            # {"id": 3, "name": "tb_3", "color": np.array([0,0,1])},  # Blue color for the third link
            # Add more job entries as needed
        ]
        
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
        cloner = GridCloner(spacing=3,num_per_row=18)
        sph_node_prim_path = "/World/node"
        VisualSphere(
                prim_path=sph_node_prim_path,
                name="sph_node_name",
                position=np.array([-23.5,-18.75,0.0]),
                scale=np.array([0.2,0.2,0.2]),
                color=np.array([1,1,1]),
                visible=True,
        )
        target_paths_sph = cloner.generate_paths("/World/node",180)
        print(f't:{len(target_paths_sph)}')
        num_sph_nodes = len(target_paths_sph)
        cloner.clone(source_prim_path="/World/node", prim_paths=target_paths_sph)
        
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
    
    def tb_path_receive(self, msg):
        tb_id = msg.tb_id
        data = msg.listofpath.data
        dim0_size = msg.listofpath.layout.dim[0].size
        dim1_size = msg.listofpath.layout.dim[1].size
        re_tb_path = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
        print(f'len_tb_path: {len(re_tb_path)}')
        print(f'tb_path: {re_tb_path}')
        a = [i for i in range(1,len(re_tb_path))]
        b = [i for i in range(0,len(re_tb_path)-1)]
        for i in range(len(re_tb_path)-1):
            print(f'a_y: {re_tb_path[i+1][1]}, a_x: {re_tb_path[i+1][0]}, b_y: {re_tb_path[i][1]}, b_x: {re_tb_path[i][0]}')
            pos_rot = create_link_graph(re_tb_path[i+1][1], re_tb_path[i+1][0], re_tb_path[i][1], re_tb_path[i][0])
            print(f'pos_rot: {pos_rot}')
            self.ros_world.scene.add(
                VisualCylinder(
                    position=pos_rot["pos"],
                    prim_path=f'/World/Link_{tb_id}_{i+1}',
                    name=f'link_{tb_id}_{i+1}',
                    orientation=pos_rot["rot"],
                    scale=np.array([0.1,0.1,3.0]),
                    color=self.tb_job[tb_id-1]["color"]
                )
            )
        
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

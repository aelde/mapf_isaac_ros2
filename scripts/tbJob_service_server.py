#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mapf_isaac.srv import TbJob
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
from geometry_msgs.msg import Point
from mapf_isaac.msg import TbPathtoGoal,TbTask


class TbJobServer(Node):
    def __init__(self):
        super().__init__('Tb_Job_service_server_node')
        self.srv = self.create_service(TbJob, 'tb_job_srv', self.job_respone)
        # self.pub_job_task = self.create_publisher(Float32MultiArray,'all_tb_job_task',10)
        # self.pub_job_task = self.create_publisher(MultiArrayDimension,'all_tb_job_task',10)
        self.pub_job_task = self.create_publisher(TbTask,'all_tb_job_task',10)
        self.tb_job_task = []
        self.tb_task = TbTask()
        
    def job_respone(self, request, response):
        tb_id = request.tb_id
        tb_goal = request.goal_point
        start_routing = request.start_routing
        print(f'Tb: {tb_id} job received')
        self.tb_job_task.append([tb_id, tb_goal.x,tb_goal.y,tb_goal.z])
        print(f'add Tb: {tb_id} job to task')
        
        my_res_msg = Point()
        my_res_msg.x = 1.11
        my_res_msg.y = 2.22
        my_res_msg.z = 3.33
        response.distance = my_res_msg
        print(f'response: {response}')
        print()
        print(f'all task are:')
        for i in range(len(self.tb_job_task)): print(self.tb_job_task[i])
        print('end task****')
        # if start_routing:
        #     print(f'publishing all tb job task....')
        #     my_msg = Float32MultiArray()
        #     # my_msg = self.tb_job_task
        #     my_msg = [[1.0,2.0,3.0,4.0],[1.1,2.2,3.3,4.4]]
        #     # for task in self.tb_job_task:
        #     #     my_msg.data.extend(task)
        #     self.pub_job_task.publish(my_msg)
        #     self.tb_job_task = []
        #     print(f'published...')
        self.tb_task.start_routing=start_routing
        self.tb_task.tb_id = tb_id
        self.tb_task.tb_goal.x = tb_goal.x
        self.tb_task.tb_goal.y = tb_goal.y
        self.tb_task.tb_goal.z = tb_goal.z
        self.pub_job_task.publish(self.tb_task)

        return response
        
def main(args=None):
    rclpy.init(args=args)
    my_pub = TbJobServer()
    print('tb job service running')
    try:
        rclpy.spin(my_pub)
    except KeyboardInterrupt:
        print('tb job service terminated')
    finally:
        my_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
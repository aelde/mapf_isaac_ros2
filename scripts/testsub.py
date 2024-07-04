#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayDimension, Float32MultiArray
from mapf_isaac.msg import TbPathtoGoal,TbTask

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('array_subscriber_node')
        self.subscription = self.create_subscription(
            TbTask,
            'tb_initial_pos',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(f'msg: {msg}')
        print()
    # def listener_callback(self, msg):
    #     tb_id = msg.tb_id
    #     data = msg.listofpath.data
    #     if not msg.listofpath.layout.dim:
    #         self.get_logger().error('Received message with empty dimensions')
    #         return
        
    #     dim0_size = msg.listofpath.layout.dim[0].size
    #     dim1_size = msg.listofpath.layout.dim[1].size
    #     reconstructed_data = [data[i * dim1_size:(i + 1) * dim1_size] for i in range(dim0_size)]
            
    #     self.get_logger().info(f'I heard: tb:{tb_id}, data:{reconstructed_data}')
    #     print(f'reconstructed is: ')
    #     for i in reconstructed_data: print(i)
    #     print()

def main(args=None):
    rclpy.init(args=args)
    my_sub = SubscriberNode()
    print('Subscriber node running')
    try:
        rclpy.spin(my_sub)
    except KeyboardInterrupt:
        print('Subscriber node terminated')
    finally:
        my_sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

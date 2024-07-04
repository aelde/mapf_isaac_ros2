#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MultiRobotCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('multi_robot_cmd_vel_publisher')
        
        # Create publishers for each robot
        self.robots = ['robot1', 'robot2', 'robot3']
        self.pub = {}
        
        for robot in self.robots:
            topic = f'/{robot}/cmd_vel'
            self.pub[robot] = self.create_publisher(Twist, topic, 10)
            self.get_logger().info(f'Publisher created for {topic}')
        
        # Timer to publish cmd_vel periodically
        self.timer = self.create_timer(0.5, self.publish_cmd_vel)
        
    def publish_cmd_vel(self):
        # Create a Twist message
        msg = Twist()
        msg.linear.x = 0.5  # Linear velocity
        msg.angular.z = 0.1  # Angular velocity
        
        # Publish the message to each robot
        for robot, publisher in self.pub.items():
            publisher.publish(msg)
            self.get_logger().info(f'Published cmd_vel to {robot}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotCmdVelPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

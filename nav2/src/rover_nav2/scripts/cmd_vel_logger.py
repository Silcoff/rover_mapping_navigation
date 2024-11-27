#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_logger')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # prevent unused variable warning

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        turning_radius = self.calculate_turning_radius(linear_x, angular_z)
        self.get_logger().info(f'Linear: {linear_x}, Angular: {angular_z}, Turning Radius: {turning_radius}')

    def calculate_turning_radius(self, linear_x, angular_z):
        if angular_z == 0:
            return float('inf')  # Straight line
        return linear_x / angular_z

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
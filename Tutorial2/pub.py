#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import random

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'numbers', 10)
        timer_period = 1.0  # publish every 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Number Publisher started...")

    def timer_callback(self):
        msg = Int32MultiArray()
        # publish two random numbers between 1 and 10
        msg.data = [random.randint(1, 10), random.randint(1, 10)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing numbers: {msg.data[0]}, {msg.data[1]}')

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

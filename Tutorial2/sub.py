#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class NumberAdder(Node):
    def __init__(self):
        super().__init__('number_adder')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'numbers',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Number Adder started...")

    def listener_callback(self, msg):
        if len(msg.data) == 2:
            a, b = msg.data
            result = a + b
            self.get_logger().info(f'Received: {a}, {b} | Sum = {result}')
        else:
            self.get_logger().warn("Message does not contain exactly 2 numbers.")

def main(args=None):
    rclpy.init(args=args)
    node = NumberAdder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles


class OpenLoopController(Node):

    def __init__(self):
        super().__init__('open_loop_controller')

        publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        msg = DutyCycles()
        msg.duty_cycle_left = 1.0
        msg.duty_cycle_right = 1.0

        while True:
            publisher.publish(msg)

        
        # TODO: Implement
        
    # TODO: Implement


def main():
    rclpy.init()
    node = OpenLoopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
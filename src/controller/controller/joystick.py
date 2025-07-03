#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles
from sensor_msgs.msg import Joy


class Joystick(Node):

    def __init__(self):
        super().__init__('joystick')
        subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Max speeds
        self.v_max = 1.0
        self.w_max = 1.0

        # Publisher
        self.publisher = self.create_publisher(Twist, '/motor_controller/twist', 10)
        self.publisherTest = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)


    def joy_callback(self, data):
        linear = data.axes[1]
        anuglar = data.axes[0]

        msg = Twist()
        msg.linear.x = linear * self.v_max
        msg.angular.z =  anuglar * self.w_max
        print(msg)
        self.publisher.publish(msg)

        # """Direct control XDD"""
        # pwm = DutyCycles()
        # pwm.duty_cycle_left = data.axes[1]
        # pwm.duty_cycle_right = data.axes[3]
        # print(f"{pwm.duty_cycle_left}||||{pwm.duty_cycle_right}")
        # self.publisherTest.publish(pwm)



def main():
    rclpy.init()
    node = Joystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
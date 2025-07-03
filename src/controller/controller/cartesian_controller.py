#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles, Encoders
from geometry_msgs.msg import Twist

import math


class CartesianController(Node):

    def __init__(self):
        super().__init__('cartesian_controller')

        # Init attributes
        self.v_desired, self.w_desired, self.delta_encoder_left, self.delta_encoder_right = 0, 0, 0, 0
        self.int_error_w1, self.int_error_w2 = 0, 0

        # Constants
        self.b = 0.31
        self.r = 0.04921
        self.f = 10
        self.ticks_per_rev = 48 * 64
        self.dt = 0.1 #50 / 1000 

        # Control parameters
        self.alpha_w1, self.alpha_w2 = 0.66, 0.6
        self.beta_w1, self.beta_w2 = 0.005, 0.002 #0.005, 0.002


        # self.alpha_w1, self.alpha_w2 = 0.9, 0.8
        # self.beta_w1, self.beta_w2 = 0.5, 0.5

        # Subscribers
        speed_subscriber = self.create_subscription(Twist, '/motor_controller/twist', self.speed_callback, 10)
        encoders_subscriber = self.create_subscription(Encoders, '/motor/encoders', self.encoders_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        self.msg = DutyCycles()

        # Create a timer for periodic updates
        self.timer = self.create_timer(self.dt, self.update)
        
    def update(self):

        # Calculate desired values using the kinematic model
        v_w2_desired = self.v_desired + self.b * self.w_desired
        v_w1_desired = self.v_desired - self.b * self.w_desired

        # Calculate estimated values
        v_w2_estimated = 2 * math.pi * self.r * self.f *  self.delta_encoder_right / self.ticks_per_rev
        v_w1_estimated = 2 * math.pi * self.r * self.f * self.delta_encoder_left / self.ticks_per_rev

        # Controller, right wheel
        error_w2 = v_w2_desired - v_w2_estimated
        self.int_error_w2 = self.int_error_w2 + error_w2


        # Controller, left wheel
        error_w1 = v_w1_desired - v_w1_estimated
        self.int_error_w1 = self.int_error_w1 + error_w1

        no_pid = False
        if no_pid:
            self.msg.duty_cycle_right = v_w2_desired
            self.msg.duty_cycle_left = v_w1_desired
        else:
            self.msg.duty_cycle_right = self.alpha_w2 * error_w2 + self.beta_w2 * self.int_error_w2 * self.dt
            self.msg.duty_cycle_left = self.alpha_w1 * error_w1 + self.beta_w1 * self.int_error_w1 * self.dt

        # Check that the values are inside [-1 ,1]
        if self.v_desired == 0.0 and self.w_desired == 0.0:
            self.msg.duty_cycle_left = 0.0
            self.msg.duty_cycle_right = 0.0
        else:
            self.msg.duty_cycle_left = float(max(self.msg.duty_cycle_left, -1))
            self.msg.duty_cycle_left = float(min(self.msg.duty_cycle_left, 1))
            self.msg.duty_cycle_right = float(max(self.msg.duty_cycle_right, -1))
            self.msg.duty_cycle_right = float(min(self.msg.duty_cycle_right, 1))
        #print(f"{self.msg.duty_cycle_left}||||{self.msg.duty_cycle_right}")

        # Publish the pwm
        self.publisher.publish(self.msg)
        # print(self.msg)

        # Estimate velocity
        v = (v_w1_estimated + v_w2_estimated)/2
        w = (v_w2_estimated - v_w1_estimated)/(2*self.b)

        # print(error_w1, "\t\t", error_w2)
 

    def speed_callback(self, data):
        self.v_desired = data.linear.x
        self.w_desired = data.angular.z

        print(self.v_desired, self.w_desired)

    def encoders_callback(self, data):
        # print(data.delta_encoder_left)
        self.delta_encoder_left = data.delta_encoder_left
        self.delta_encoder_right = data.delta_encoder_right

def main():
    rclpy.init()
    node = CartesianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
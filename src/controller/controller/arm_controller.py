#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles, Encoders
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray, Float32
from sensor_msgs.msg import JointState
from custom_messages.msg import DetectedObject

from math import cos, sin, pi
import numpy as np
from random import *
from time import sleep


class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')
        print("init arm controller")
        self.arm_publisher = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        sleep(3)
        self.l3 = 0.169
        self.l4 = 0.094
        self.l5 = 0.101

        self.motion_time = 1000     # ms
        

        # print(self.get_clock().now().seconds_nanoseconds())
        # sleep(1)
        # print(self.get_clock().now().seconds_nanoseconds())
        # # print(type(rclpy.time.Time().seconds_nanoseconds()))
        # print(type(self.get_clock().now().seconds_nanoseconds()))
        # print(self.get_clock().now().seconds_nanoseconds()[0])

        self.msg = Int16MultiArray() # old values: 6000,12000,4000,16000,10000,12000
        self.arm_pose = [0,12000,5000,19000,10000,12000, 2000,2000,2000,2000,2000,2000]
        self.msg.data = self.arm_pose
        self.arm_publisher.publish(self.msg)

        self.arm_pose[6:] = [self.motion_time] * 6 

        self.grasp_threshold = -0.155

        
        self.u, self.v = 0, 0                       # Current middle pixel of the detected object
        self.u_tilde, self.v_tilde = 300, 400           # Target pixel, which will be inside the gripper.
        self.theta = 0
        self.theta_list = []

        # 300 460

        # Pid controller parameters
        self.alpha_v, self.alpha_u = 10, 3

        sleep(4)
        print('sleep end')
        self.last_time = 0

        self.subscriber = self.create_subscription(DetectedObject, '/detected_object', self.object_detection, 10)
        self.kinematics_subs = self.create_subscription(JointState, '/servo_pos_publisher', self.forward_kinematics, 10)
        self.kinematics_subs = self.create_subscription(Float32, '/object_orientation', self.orientation_update, 10)


        self.v_timer = self.create_timer(1, self.v_controller)
        self.u_timer = self.create_timer(1, self.u_controller)

        # self.f_kinemactics = self.create_timer(0.5, self.forward_kinematics)
        self.stop_grasping_motion = False
        self.first_time = True
        self.gripper_rotated = False


        # ros2 topic pub /multi_servo_cmd_sub --once std_msgs/Int16MultiArray "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [6000,-1,2000,10000,4000,-1,2000,2000,2000,2000,2000,2000]}"


    def object_detection(self, data):
        self.u = data.u
        self.v = data.v
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0]

    def orientation_update(self, data):
        while len(self.theta_list) < 50:
            self.theta_list.append(data.data)

        if len(self.theta_list) == 50:
            # Calculate angle
            average_theta = sum(self.theta_list) / len(self.theta_list)
            # average_theta = abs(average_theta)
            average_theta = 90.0 - average_theta
            self.theta = int(- average_theta * 100)
            print("################# AVERAGE THETA:", self.theta)


    def v_controller(self):
        time_now = self.get_clock().now().seconds_nanoseconds()[0]
        if time_now - self.last_time < 5:
        # Move motor 5 down at a constant speed
            if self.arm_pose[4] >= 4500 and not self.stop_grasping_motion:
                self.arm_pose[4] -= 400
                
                # P controller for motor 4
                error_v = self.v - self.v_tilde
                motor_4_change = error_v * self.alpha_v
                motor_4_change = max(motor_4_change, -1500)

                self.arm_pose[3] += motor_4_change

                # Publish controll message
                self.msg.data = self.arm_pose
                self.arm_publisher.publish(self.msg)

            elif self.arm_pose[3] <= 18000 and not self.stop_grasping_motion:

                if self.first_time and self.arm_pose[3] < 12000:
                    self.arm_pose[3] = 12000

                    self.first_time = False

                self.grasp_threshold = -0.135
                self.arm_pose[3] += 200
                # self.arm_pose[4] = -1

                
                # P controller for motor 4
                error_v = self.v - self.v_tilde
                motor_4_change = error_v * 10
                self.arm_pose[2] -= motor_4_change

                # Publish controll message
                self.msg.data = self.arm_pose
                self.arm_publisher.publish(self.msg)
        
    def u_controller(self):
        # P controller for the wheels
        # Move motor 5 down at a constant speed
        time_now = self.get_clock().now().seconds_nanoseconds()[0]
        if time_now - self.last_time < 5:
            if  not self.stop_grasping_motion:
                
                # P controller for motor 4
                error_u = self.u_tilde - self.u
                motor_6_change = error_u * self.alpha_u
                self.arm_pose[5] += motor_6_change

                # Publish controll message
                self.msg.data = self.arm_pose
                self.arm_publisher.publish(self.msg)

    def forward_kinematics(self, data):
        # q -> X
        # X = [x y]
        # q = [q3 q4 q5]

        l3, l4, l5 = self.l3, self.l4, self.l5
        q3, q4, q5 = data.position[2], data.position[3], data.position[4]

        q3 = (q3-12000) / 100 * pi / 180 
        q4 = (q4-12000) / 100 * pi / 180
        q5 = (q5-3000) / 100 * pi / 180

        y = l5 * sin(q5) + l4 * sin(q5-q4) + l3 * sin(q5-q4+q3)
        x = l5 * cos(q5) + l4 * cos(q5-q4) + l3 * cos(q5-q4+q3)

        X = np.array([[x],
                      [y]])
        print("effector pose", y)
        print(self.arm_pose)


        if y < -0.1:
            self.v_tilde = 430      # 460

            # Thresholds:
            # - cube and sphere: y < -0.158
            # - Fluffy animals: y < -0.14       because we need to rotate the gripper and then grasp


            if y < -0.14 or (self.arm_pose[3] > 17600 and self.arm_pose[2]> 12000):
                print("grasp")
                print("effector pose", y)

                self.grasp()


            #            if y < -0.158 or (self.arm_pose[3] > 17600 and self.arm_pose[2]> 12000):



    def grasp(self):

        # Rotate gripper
        self.arm_pose[1] += self.theta
        self.arm_pose[3] += 1000        # Only if the case of fluffy animals
        self.arm_pose
        self.msg.data = self.arm_pose
        self.arm_publisher.publish(self.msg)
        sleep(2)

        # Grasp
        self.stop_grasping_motion = True
        self.arm_pose[0] = 12000        # 11000
        self.msg.data = self.arm_pose
        self.arm_publisher.publish(self.msg)
        sleep(2)

        # Lift up
        self.arm_pose = [-1,-1,-1,-1,6000,-1, self.motion_time,self.motion_time,self.motion_time,self.motion_time,self.motion_time,self.motion_time]
        self.msg.data = self.arm_pose
        self.arm_publisher.publish(self.msg)
        sleep(2)

        # Go back to pre-grasp
        self.arm_pose = [-1,12000,5000,19000,10000,12000, self.motion_time,self.motion_time,self.motion_time,self.motion_time,self.motion_time,self.motion_time]
        self.msg.data = self.arm_pose
        self.arm_publisher.publish(self.msg)
        exit()


def main():
    rclpy.init()
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from math import atan2,exp,sqrt
from geometry_msgs.msg import Twist, PointStamped
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs

 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 

from tf2_ros import TransformException 

import math

class ObjectFollowingController(Node):

    def __init__(self):
        super().__init__('object_following_controller')
        self.x_target= 0.0
        self.y_target=0.0
        self.dist=0.0
        self.yaw_error_int = 0.0
        #pose
        self.x_odometry = 0.0
        self.y_odometry = 0.0
        self.yaw_odometry = 0.0

        self.yaw_tilde = 0.0

        self.pub = self.create_publisher(Twist,'/motor_controller/twist',10)
        timer_period=0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.adc_subscriber = self.create_subscription(PointStamped,'/camera/object/point',self.point_callback,10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_timer = self.create_timer(timer_period, self.odom_timer_callback)

        
    def timer_callback(self):
        self.dist= sqrt((self.x_target-self.x_odometry)**2 + (self.y_target-self.y_odometry)**2)
        self.yaw_tilde = atan2(self.y_target-self.y_odometry,self.x_target-self.x_odometry)
        alpha_w =  1.2
        beta_w = 0.1
        alpha_v = 0.00
        beta_v = 0.4
        t = Twist()
        t.linear.x = beta_v * self.dist
        # print(self.yaw_tilde, "yaw_tilde")
        # print(self.dist, 'dist')
        if t.linear.x<0.05 and self.dist!=0.0:
            t.linear.x=0.05
        elif self.dist<0.2:
            t.linear.x = 0.0
        elif t.linear.x > 0.5:
            t.linear.x = 0.5
        elif self.dist > 0.3:
            t.linear.x = 0.5
            # print("approach done!")
            # exit()

        yaw_error = self.yaw_tilde - self.yaw_odometry
        self.yaw_error_int += yaw_error

        t.angular.z = alpha_w*yaw_error + beta_w*self.yaw_error_int

        self.pub.publish(t)
      

    def point_callback(self,data:PointStamped):
        
        if data.header.frame_id!='NIF':
            try:
                t=self.tf_buffer.lookup_transform('odom',data.header.frame_id,data.header.stamp)
                p=tf2_geometry_msgs.do_transform_point(data,t)
            except:
                print('Could not find camera to odom')
                return

            self.x_target = p.point.x
            self.y_target = p.point.y
            print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            print(self.x_target, self.y_target, "target xy")

        '''else:
            self.dist = 0.0
            self.y_target = 0.0'''



    def odom_timer_callback(self):
        trans = None
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        "odom",
                        "base_link",
                        now)
        except:
            print('Could not find transform odom to base_link')
            return
        

        self.x_odometry = trans.transform.translation.x
        self.y_odometry = trans.transform.translation.y    
        roll, pitch, yaw = self.euler_from_quaternion(
        trans.transform.rotation.x,
        trans.transform.rotation.y,
        trans.transform.rotation.z,
        trans.transform.rotation.w)      
        self.yaw_odometry = yaw    
        # print(self.x_odometry, self.y_odometry, self.yaw_odometry)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians

def main():
    rclpy.init()
    node = ObjectFollowingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
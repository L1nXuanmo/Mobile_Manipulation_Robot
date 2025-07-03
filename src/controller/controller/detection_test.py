#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from open3d import open3d as o3d


import ctypes
import struct

import time
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Quaternion, Pose, Point
from sensor_msgs.msg import Image
from std_msgs.msg import Header, ColorRGBA
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from custom_messages.msg import DetectedObject
import cv2

from builtin_interfaces.msg import Time

from cv_bridge import CvBridge




class DetectionTest(Node):

    def __init__(self):
        super().__init__('detection_test')

        # Initialize the publisher
        self._pub = self.create_publisher(DetectedObject, '/detected_object', 10)

        # Subscribe to image topic
        self.create_subscription(Image, '/image_raw', self.detection_callback, 10)
        
        self.cv_bridge = CvBridge()

    def detection_callback(self, data):
        # print(data)
        # Create CV bridge, and convert into HSV
        img = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        # print(img.shape)
        # cv2.imwrite("/home/group8/Pictures/test.png", img)

        
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # cv2.imshow("img", img)
        # cv2.waitKey(0)

        # Filter red pixels
        # print(img[:,:,0])
        red_img = cv2.inRange(img, (165,30,100),(178,255,255))

        if cv2.countNonZero(red_img) > 1500:

            kernel = np.ones((5,5))
            red_img = cv2.morphologyEx(red_img, cv2.MORPH_OPEN, kernel)
            object_msg = DetectedObject()
            # img_show = red_img*100
            # Calculate centroid of the detected object
            moments = cv2.moments(red_img)
            div = moments["m00"]
            if div != 0:
                print(moments["m01"]/div, moments["m10"]/div)
                object_msg.u = int(moments["m10"]/div)
                object_msg.v = int(moments["m01"]/div)
                self._pub.publish(object_msg)
                
                # cv2.imshow("detected object", img_show)
                # cv2.waitKey(100)
        
  

def main():
    rclpy.init()
    node = DetectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import sensor_msgs_py.point_cloud2 as pc2
from open3d import open3d as o3d

import cv2
from cv_bridge import CvBridge


import ctypes
import struct

import time
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Quaternion, Pose, Point
from std_msgs.msg import Header, ColorRGBA, Float32
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs

from custom_messages.msg import DetectedObject

from builtin_interfaces.msg import Time
from sklearn.cluster import KMeans

"""
ros2 launch robp_boot_camp_launch boot_camp_part3_launch.xml

ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom

ros2 run detection detection

ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/bags/boot_camp
"""



class FluffyDetection(Node):

    def __init__(self):
        super().__init__('fluffy_detection')

        print("FLUFFY DETECTION")
        

        # Initialize the publisher
        self._pub = self.create_publisher(DetectedObject, '/detected_object', 10)
        self.orientation_pub = self.create_publisher(Float32, '/object_orientation', 10)

        # Subscribe to point cloud topic and call callback function on each recieved message
        self.create_subscription(
            Image, '/image_raw', self.image_callback, 1000)
        
        # Init variables
        self.image = None
        self.cv_bridge = CvBridge()
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=10)

        


    def image_callback(self, data):

        # Convert to OpenCV
        self.image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")


        self.image = self.image[:445,:,:]

        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)


        # _, binary_mask = cv2.threshold(self.image, 127, 200, cv2.THRESH_BINARY)

        # kernel = np.ones((3,3))
        # binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, kernel)

        # # Apply Gaussian blur to reduce noise

        # Remove black pixels (gripper)
        # _, self.image = cv2.threshold(self.image, 0, 240, cv2.THRESH_BINARY)


        blurred_image = cv2.GaussianBlur(self.image, (9, 9), 0)
        edges = cv2.Canny(blurred_image, threshold1=30, threshold2=150)  # Adjust thresholds as needed
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # Kernel size can be adjusted
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)


        # # Find edge coordinates
        # edge_coordinates = np.argwhere(edges > 0)  # Get coordinates of non-zero pixels

        # # Perform K-means clustering
        # kmeans = KMeans(n_clusters=2)
        # kmeans.fit(edge_coordinates)

        # # Get labels and cluster centers
        # labels = kmeans.labels_
        # cluster_centers = kmeans.cluster_centers_

        # # Calculate the density of each cluster
        # density_0 = np.sum(labels == 0)
        # density_1 = np.sum(labels == 1)

        # # Select the densest cluster
        # if density_0 > density_1:
        #     densest_cluster_label = 0
        #     less_dense_cluster_label = 1
        # else:
        #     densest_cluster_label = 1
        #     less_dense_cluster_label = 0

        # # Remove less dense cluster
        # edges_filtered = np.zeros_like(edges)
        # for label, (x, y) in zip(labels, edge_coordinates):
        #     if label == densest_cluster_label:
        #         edges_filtered[x, y] = 255



        binary_mask = edges


        object_msg = DetectedObject()
        orientation = Float32()

        moments = cv2.moments(binary_mask)
        div = moments["m00"]
        if div != 0:
            u = int(moments["m10"]/div)
            v = int(moments["m01"]/div)

            # Calculate central moments
            mu_20 = moments['mu20'] / moments['m00']
            mu_02 = moments['mu02'] / moments['m00']
            mu_11 = moments['mu11'] / moments['m00']

            # Calculate orientation
            theta = 0.5 * np.arctan2(2 * mu_11, mu_20 - mu_02)

            # Convert orientation from radians to degrees
            theta_deg = np.degrees(theta)

            # Define line length
            line_length = 50

            # Calculate endpoint coordinates
            x1 = int(u - line_length * np.cos(theta))
            y1 = int(v - line_length * np.sin(theta))
            x2 = int(u + line_length * np.cos(theta))
            y2 = int(v + line_length * np.sin(theta))

            cv2.line(binary_mask, (x1, y1), (x2, y2), (255, 0, 0), 2)
                
            cv2.circle(binary_mask, (u,v), 5, (255,0,0), -1)

            print(theta_deg)

            object_msg.u = u
            object_msg.v = v
            orientation.data = theta_deg
            self._pub.publish(object_msg)
            self.orientation_pub.publish(orientation)

        cv2.imshow("seg", binary_mask)




        # Exit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()






def main():
    rclpy.init()
    node = FluffyDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
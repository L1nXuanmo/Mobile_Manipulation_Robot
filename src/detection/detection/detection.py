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
from std_msgs.msg import Header, ColorRGBA
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs


from builtin_interfaces.msg import Time

"""
ros2 launch robp_boot_camp_launch boot_camp_part3_launch.xml

ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom

ros2 run detection detection

ros2 bag play --read-ahead-queue-size 100 -l -r 1.0 --clock 100 --start-paused ~/dd2419_ws/bags/boot_camp
"""



class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher
        self._pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/ds_points', 10)
        
        self.point_pub = self.create_publisher(
            PointStamped, '/camera/object/point',10
        )

        # Subscribe to point cloud topic and call callback function on each recieved message
        self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.cloud_callback, 10)
        
        self.buffer = Buffer(cache_time=None)
        self.tf_listener = TransformListener(self.buffer, self)
        
        self.fig, self.ax = plt.subplots()
        self.broadcaster = TransformBroadcaster(self)



    def cloud_callback(self, msg: PointCloud2):
        """Takes point cloud readings to detect objects.

        This function is called for every message that is published on the '/camera/depth/color/points' topic.

        Your task is to use the point cloud data in 'msg' to detect objects. You are allowed to add/change things outside this function.

        Keyword arguments:
        msg -- A point cloud ROS message. To see more information about it 
        run 'ros2 interface show sensor_msgs/msg/PointCloud2' in a terminal.
        """
        
        # Convert ROS -> NumPy
        gen = pc2.read_points_numpy(msg, skip_nans=True)
        xyz = gen[:,:3]
        rgb = np.empty(xyz.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f' , c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            rgb[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8) 
            rgb[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8) 
            rgb[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        rgb = rgb.astype(np.float32) / 255

        # Convert NumPy -> Open3D
        cloud = o3d.geometry.PointCloud()    
        cloud.points = o3d.utility.Vector3dVector(xyz)
        cloud.colors = o3d.utility.Vector3dVector(rgb)

        # Downsample the point cloud to 5 cm
        # ds_cloud = cloud.voxel_down_sample(voxel_size=0.1)

        # Convert Open3D -> NumPy
        #points = np.asarray(ds_cloud.points)
        #colors = np.asarray(ds_cloud.colors)
        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)

        threshold = 0.8
        point_norms = np.linalg.norm(points, axis=1)

        close_points = points[point_norms<threshold]
        close_colors = colors[point_norms<threshold]

        red_mask = (close_colors[:, 0] > 0.6) & \
                    (close_colors[:, 1] < 0.4) & \
                    (close_colors[:, 2] < 0.4)
        
        
        red_points = close_points[red_mask]
        red_colors = close_colors[red_mask]



        self.get_logger().info("RED DETECTED") if red_points.shape[0]!=0 else None

        if red_points.shape[0]!=0:
            point = np.mean(red_points, axis=0)
            p = PointStamped()
            p.header = Header()
            p.header.frame_id='camera_depth_optical_frame'
            p.header.stamp = msg.header.stamp
            p.point.x = point[0]
            p.point.y = point[1]
            p.point.z = point[2]
            self.point_pub.publish(p)
        else:
            p = PointStamped()
            p.header = Header()
            p.header.frame_id = 'NIF'
            self.point_pub.publish(p)
        stamp = msg.header.stamp
        
        red_msg = self.open3d_to_ros_pointcloud2(red_points, red_colors, stamp)
        
        self._pub.publish(red_msg)

    def open3d_to_ros_pointcloud2(self, points, colors, stamp):
        colors = colors * 255

        # Header
        header = Header()
        header.frame_id = 'camera_depth_optical_frame'      #camera_depth_optical_frame
        header.stamp = stamp

        # Define fields
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.FLOAT32, count=1)
        ]

        data_points =np.zeros((colors.shape[0],4))

        for idx, color in enumerate(colors):
            rgb = struct.unpack('I', struct.pack('BBBB', int(color[0]), int(color[1]), int(color[2]), 255))[0]
            data_points[idx,:] = np.array([points[idx,0], points[idx,1], points[idx,2], rgb])


        pc2_msg = pc2.create_cloud(header, fields, data_points)

        return pc2_msg
    




def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
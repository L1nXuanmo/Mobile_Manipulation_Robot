#!/usr/bin/env python3

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Python standard library
from math import sin, cos

# Numpy
import numpy as np
import pandas as pd

# "Local version" of ROS messages
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid


def quaternion_from_euler(roll, pitch, yaw):
    q = [0]*4

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)

    q[0] = cy * sr * cp - sy * cr * sp
    q[1] = cy * cr * sp + sy * sr * cp
    q[2] = sy * cr * cp - cy * sr * sp
    q[3] = cy * cr * cp + sy * sr * sp

    return q


class GridMap:
    def __init__(self,frame_id="map", resolution=0.05, width=5, height=5,
                 map_origin_x=0, map_origin_y=0, map_origin_yaw=0, 
                 default_value=np.int8(-1)):
        self.__frame_id = frame_id
        self.__resolution = resolution
        self.__width = width
        self.__height = height

        q = quaternion_from_euler(0, 0, map_origin_yaw)
        p = Pose()
        p.position.x=map_origin_x
        p.position.y = map_origin_y
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        self.__origin = p

        # default_value = unknown space (-1)
        self.__map = np.full((height, width), default_value, dtype=int)
        

    
    def __getitem__(self, pos):
        x, y = pos
        return self.__map[y, x]

    def __setitem__(self, pos, val):
        x, y = pos
        self.__map[int(y), int(x)] = val

    def get_resolution(self):
        return self.__resolution

    def get_width(self):
        return self.__width

    def get_height(self):
        return self.__height

    def get_origin(self):
        return self.__origin

    def to_message(self):
        map = OccupancyGrid()

        # Fill in the header
        map.header.frame_id = self.__frame_id

        # Fill in the info
        map.info.resolution = self.__resolution
        map.info.width = self.__width
        map.info.height = self.__height
        map.info.origin = self.__origin

        d = self.__map.reshape(-1).tolist()
        # Fill in the map data
        map.data = d # (self.__map.size)

        return map

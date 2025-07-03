
# Python standard library
import copy
import numpy as np
import pandas as pd

# ROS
import rclpy
from rclpy.node import Node
import message_filters

# ROS messages
from geometry_msgs.msg import PoseStamped as PoseStampedROS
from sensor_msgs.msg import LaserScan as LaserScanROS
from nav_msgs.msg import OccupancyGrid as OccupancyGridROS
from nav_msgs.msg import Path, Odometry
from map_msgs.msg import OccupancyGridUpdate as OccupancyGridUpdateROS
from std_msgs.msg import Header



from . grid_map import GridMap
from . mapping import Mapping


class Mapper(Node):
    def __init__(self):
        super().__init__('Mapper')
        # Parameters
        ws = self.get_workspace()
        max_ws = ws.max()
        min_ws = ws.min()
        map_frame_id = "odom"
        map_resolution = 0.05
        map_width = int((max_ws.x-min_ws.x)/map_resolution)
        map_height = int((max_ws.y-min_ws.y)/map_resolution)
        map_origin_x = min_ws.x
        map_origin_y = min_ws.y
        map_origin_yaw = 0

        inflate_radius = int(0.2/map_resolution)

        unknown_space = np.int8(-1)
        free_space = np.int8(0)
        c_space = np.int8(50)
        occupied_space = np.int8(100)
        out_of_bounds = np.int8(-2)

        unknown_space_rgb = (128, 128, 128)  # Grey
        free_space_rgb = (255, 255, 255)     # White
        c_space_rgb = (255, 0, 0)            # Red
        occupied_space_rgb = (255, 255, 0)   # Yellow
        wrong_rgb = (0, 0, 255)              # Blue
        optional=None
        self.__pose = None
        self.__map = GridMap(map_frame_id, map_resolution, map_width, map_height,
                         map_origin_x, map_origin_y, map_origin_yaw)

        self.__inflated_map = self.__map

        self.__mapping = Mapping(unknown_space, free_space, c_space,
                                 occupied_space,out_of_bounds, inflate_radius, ws,optional)
        
        self.__mapping.set_workspace(self.__map)
        
        self.__odom_sub = message_filters.Subscriber(self,Path, '/path')
        self.__scan_sub = message_filters.Subscriber(self, LaserScanROS,'/scan')


        self.__ts = message_filters.ApproximateTimeSynchronizer([self.__odom_sub,
                                                     self.__scan_sub], 10, 0.01)
        self.__ts.registerCallback(self.callback)

        self.__map_pub = self.create_publisher(OccupancyGridROS,'/map',1)
     
        self.__map_updates_pub = self.create_publisher(OccupancyGridUpdateROS,'/map_updates',10)
      

        self.__map_inflated_pub = self.create_publisher(OccupancyGridROS,'/map_grid',1)

        self.publish_map()


    def callback(self, path, scan_ros):
        scan = scan_ros
        pose = path.poses[-1]
        self.__map, update = self.__mapping.update_map(self.__map, pose, scan)

        if isinstance(update, OccupancyGridUpdateROS) and len(update.data) != 0:
            self.publish_map_update(update)
            map = copy.deepcopy(self.__map)
            self.__inflated_map = self.__mapping.inflate_map(map)
            self.publish_inflated_map()
      
            self.publish_map()

    def publish_map(self):
        # Get ROS occupancy map
    
        map = self.map_to_message(self.__map)
        self.__map_pub.publish(map)
    def publish_map_update(self, update):
        # Get ROS occupancy map updatframe_ide
        update_ros = self.map_update_to_message(update)
        # Only send out the update
        self.__map_updates_pub.publish(update_ros)

    def publish_inflated_map(self):
        # Get ROS occupancy map
        map = self.map_to_message(self.__inflated_map)
        self.__map_inflated_pub.publish(map)

    def map_to_message(self, map):
        '''
        :type map: Map
        '''
        map = map.to_message()  # OccupancyGrid
        map_ros = OccupancyGridROS()

        # Fill in the header
        map_ros.header.stamp = self.get_clock().now().to_msg()
        map_ros.header.frame_id = map.header.frame_id

        # Fill in the info
        map_ros.info.resolution = map.info.resolution
        map_ros.info.width = map.info.width
        map_ros.info.height = map.info.height
        map_ros.info.origin = map.info.origin

        # Fill in the map data
        map_ros.data = map.data

        return map_ros

    def map_update_to_message(self, update):
        '''
        :type update: OccupancyGridUpdate
        '''
        update_ros = OccupancyGridUpdateROS()

        # Fill in the header
        update_ros.header.stamp =update.header.stamp
        update_ros.header.frame_id = update.header.frame_id

        update_ros.x = update.x
        update_ros.y = update.y
        update_ros.width = update.width
        update_ros.height = update.height
        update_ros.data = update.data

        return update_ros
    
    def get_workspace(self):
        coords = pd.read_csv('~/Project/src/mapping/mapping/workspace/workspace.tsv',sep='\t')
        return coords

    
def main():
    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
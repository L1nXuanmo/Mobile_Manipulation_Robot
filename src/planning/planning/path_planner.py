#!/usr/bin/env python


# from dubins import *
from random import *
from math import *
import numpy as np
from time import *

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from nav_msgs.msg import Path, OccupancyGrid
import tf2_ros
import tf2_geometry_msgs


from geometry_msgs.msg import Twist, Pose2D, Polygon, PoseStamped, PoseArray, Pose, PointStamped
from visualization_msgs.msg import MarkerArray, Marker

from tf_transformations import quaternion_from_euler

"""
A* is used
"""

DT = 0.01
WIN_RADIUS = 0.125
STEP = 0.1

class PlanningNode():

    """ A class that defines a planning node in the A* algorithm """
    
    def __init__(self, coordinates, yaw = None, parent = None, control_signals_from_parent = [], cost_to_come = None, cost_to_go = None, visited  = False):
        self.coordinates = coordinates
        self.yaw = yaw
        self.parent = parent
        self.control_signals_from_parent = control_signals_from_parent
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.cost = self.cost_to_go #+ 0.1 * self.cost_to_come
        self.visited = visited


    def __str__(self):
        return "[" + str(self.coordinates[0]) + ", " + str(self.coordinates[1]) + "]" #+ "\n\t" + str(self.parent)
      

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Create subscribers for target pose and initial pose
        self.create_subscription(PoseStamped, '/init_pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Path,'/path',self.odom_callback,10)

        # Create subscribers for map/ occupancy grip
        self.create_subscription(OccupancyGrid, '/map_grid', self.map_callback, 1000)

        # Constant linear speed (for now)
        self.v = 0.01

        # For troubleshooting
        self.pose_array_pub = self.create_publisher(PoseArray, 'vizualize_poses', 10000)
        self.points_to_publish = PoseArray()
        self.points_to_publish.header.frame_id = "odom"
        self.pose_array_pub.publish(self.points_to_publish)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Init variables (map frame)
        self.yaw0 = 0.0
        self.xt = None
        self.yt = 0
        self.cur_pose=None

        # Robot path
        self._path_pub = self.create_publisher(Path, "/planned_path", 100)
        self._path = Path()
        self.path_list = []

        # Create a test occ grid
        self.map_height = None
        self.map_width = None
        self.resolution = 0.1
        self.map_origin = np.array([0,0])
        self.map = None
        
        # Begin solution
        self.solution_timer = self.create_timer(1, self.solution)
        

    def odom_callback(self,data:Path):
        self.cur_pose=data.poses[-1].pose
        self.x0 = int(self.cur_pose.position.x- self.map_origin[0]/ self.resolution)
        self.y0 = int(self.cur_pose.position.y- self.map_origin[1]/ self.resolution)
    def map_callback(self, data: OccupancyGrid):
        self.map_height = data.info.height
        self.map_width = data.info.width
        self.resolution = data.info.resolution
        self.map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])

        self.map = np.array(data.data)
        self.map = self.map.reshape((self.map_height, self.map_width))


    def viz(self, path_pose: PlanningNode):
        pose = Pose()
        pose.position.x = path_pose.coordinates[0] * self.resolution + self.map_origin[0]
        pose.position.y = path_pose.coordinates[1] * self.resolution + self.map_origin[1]

        q = quaternion_from_euler(0.0, 0.0, path_pose.yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.points_to_publish.poses.append(pose)

        self.pose_array_pub.publish(self.points_to_publish)


    def goal_callback(self, data:PoseStamped):
        # Target cooridnates
        print("NEW GOAL")
        try:
            self.transform = self.tf_buffer.lookup_transform('odom', 'map', self.get_clock().now().to_msg())
            pose_grid = tf2_geometry_msgs.do_transform_pose_stamped(data, self.transform)

            self.xt = int((pose_grid.pose.position.x - self.map_origin[0])/ self.resolution)
            self.yt = int((pose_grid.pose.position.y - self.map_origin[1])/ self.resolution)

            self.x0 = int(- self.map_origin[0]/ self.resolution)
            self.y0 = int(- self.map_origin[1]/ self.resolution)

            print(self.xt, self.yt)
            # return

            self.solution()
            
        except:
            print('Could not transform between odom and map')

    def pose_callback(self, data):
        # Initial cooridnates
        self.x0 = data.x
        self.y0 = data.y


    def solution(self):
        """ Find a solution using A* """
        if self.map_height!=None and self.xt!=None:
            print("START SOLUTION")

            self.reset_path()   # Remove old path

            global Q, visited_Q     # A* Queue 
            Q, visited_Q = [], []
        
            # trajectory: x, y, yaw, phi, time
            xl, yl, yawl, phil, tl = [self.x0], [self.y0], [self.yaw0], [], [0.0]

            # Init tree
            first_cost_to_go = self.calculate_cost_to_go(self.x0, self.y0, self.xt, self.yt)
            init_Node = PlanningNode(np.array([self.x0, self.y0]), self.yaw0, None, [], 0, first_cost_to_go)
            Q.append(init_Node)

            while Q != []:

                X = self.get_lowest_cost_node(Q)     # Get the element with lowest cost
                visited_Q.append(X)

                print("Q:",len(Q))


                if self.success(X):
                    # Chain backwards
                    node = X

                    while node.parent != None:
                        
                        stamp = self.get_clock().now().to_msg()
                        self.insert_pose(stamp, node.coordinates[0], node.coordinates[1], node.yaw)
                        node = node.parent

                    self._path.poses = self.path_list
                    self._path_pub.publish(self._path)
                    
                    print("SUCCESS")
                    return

                

                for delta in [[1,0], [-1,0], [0,1], [0,-1]]:
                    x, y, yaw = X.coordinates[0], X.coordinates[1], X.yaw
                    
                    # iter = 100 if phi == 0 else 157 # So that I get 90 degrees turns  

                    # for i in range(iter):
                    #     # Take small steps
                    #     x, y, yaw = self.step(x, y, yaw, phi, dt = DT)

                    #     if self.collision(x, y):
                    #         break


                    #     phis = [phi]*iter

                    x += delta[0]
                    y += delta[1]

                    if self.collision(x,y):
                        break
                    
                    else:
                        cost_to_come = X.cost_to_come + 1
                        cost_to_go = self.calculate_cost_to_go(x, y, self.xt, self.yt)

                        X_prim = PlanningNode(np.array([x,y]), yaw, X, delta, cost_to_come, cost_to_go)
                            
                        if not self.visited(X_prim, visited_Q, Q):
                            self.viz(X_prim)
                            Q.append(X_prim)

    def visited(self, point, Q_visited, Q_not_visited):
        Q_ = Q_not_visited + Q_visited
        for node in Q_:
            if round(node.coordinates[0],1) == round(point.coordinates[0],1):
                if round(node.coordinates[1],1) == round(point.coordinates[1],1):
                    return True
        
        return False
        
    def get_lowest_cost_node(self,Q):
        if Q:
            min_node = min(Q, key=lambda node: node.cost)
            Q.remove(min_node)
            return min_node
        else:
            return None
        

    def calculate_cost_to_go(self, x, y, tx, ty):
        cost_to_go = sqrt((x-tx)**2 + (y-ty)**2)
        return cost_to_go
        

    def collision(self, x, y):
        """ Checks if coordinates (x, y) is at an obsticle """

        # x_grid = int(x/self.resolution)
        # y_grid = int(y/self.resolution)
        x_grid, y_grid = x,y 

        # if x_grid < 0 or x_grid > self.map_width or y_grid <0 or y_grid > self.map_height:
        #     # Check boundaries
        #     return True


        if self.map[y_grid, x_grid] != 0:
            return True
        else:
            return False


    def success(self, c: PlanningNode):
        if c.coordinates[0] == self.xt and c.coordinates[1] == self.yt:
            return True
        else:
            return False
        
    def step(self, x, y, yaw, phi, dt):

        """ Starting from x, y, yaw, take a step during dt using control signals v and phi """

        # state rate
        dx     = STEP * cos(yaw)
        dy     = STEP * sin(yaw)
        dtheta = tan(phi)

        # new state (forward Euler integration)
        new_x     = x     + dt*dx
        new_y     = y     + dt*dy
        new_yaw = yaw + dt*dtheta

        return new_x, new_y, new_yaw
    
    def insert_pose(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the robot path and publishes the whole path. """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'map'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x * self.resolution + self.map_origin[0]
        pose.pose.position.y = y * self.resolution + self.map_origin[1]
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        try:
            transform = self.tf_buffer.lookup_transform("map", "odom", self.get_clock().now().to_msg())
            new_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)

            self.path_list.insert(0, new_pose)
        except:
            print('COuld not do transform between map and odom')

    def reset_path(self):
        self._path.poses = []
        self.path_list = []
        self.points_to_publish.poses = []

        self.pose_array_pub.publish(self.points_to_publish)
        


def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':  
    main()



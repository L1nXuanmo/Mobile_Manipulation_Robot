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


from geometry_msgs.msg import Twist, Pose2D, Polygon, PoseStamped, PoseArray, Pose
from visualization_msgs.msg import MarkerArray, Marker

from tf_transformations import quaternion_from_euler

"""
A* is used
"""

DT = 0.01
WIN_RADIUS = 0.5
STEP = 0.5

class PlanningNode():

    """ A class that defines a planning node in the A* algorithm """
    
    def __init__(self, coordinates, yaw = None, parent = None, control_signals_from_parent = [], cost_to_come = None, cost_to_go = None, visited  = False):
        self.coordinates = coordinates
        self.yaw = yaw
        self.parent = parent
        self.control_signals_from_parent = control_signals_from_parent
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.cost = 0.7 * self.cost_to_go + 0.3 * self.cost_to_come
        self.visited = visited


    def __str__(self):
        return "[" + str(self.coordinates[0]) + ", " + str(self.coordinates[1]) + "]" #+ "\n\t" + str(self.parent)
      

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Create subscribers for target pose and initial pose
        self.create_subscription(PoseStamped, '/init_pose', self.pose_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)


        # Constant linear speed (for now)
        self.v = 0.01

        # For troubleshooting
        self.pose_array_pub = self.create_publisher(PoseArray, 'vizualize_poses', 10000)
        self.points_to_publish = PoseArray()
        self.points_to_publish.header.frame_id = "grid"
        self.pose_array_pub.publish(self.points_to_publish)


        # Init variables
        self.x0 = 0.0
        self.y0 = 0.0
        self.yaw0 = 0.0
        self.xt = 9.0
        self.yt = 9.0

        # Goal
        goal = PoseStamped()
        goal.header.frame_id = "grid"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = self.xt, self.yt, 0.0 
        self.goal_publisher.publish(goal)


        # Robot path
        self._path_pub = self.create_publisher(Path, "/planned_path", 10)
        self._path = Path()
        self._path_pub.publish(self._path)

        # Create a test occ grid
        self.occ_grid_pub = self.create_publisher(OccupancyGrid, "/map_grid", 10000)
        occ_grid = OccupancyGrid()
        occ_grid.header.frame_id = "grid"
        occ_grid.info.height = 100
        occ_grid.info.width = 100
        occ_grid.info.resolution = 0.1
        self.resolution = occ_grid.info.resolution

        # data = np.zeros((int(occ_grid.info.width/occ_grid.info.resolution),int(occ_grid.info.height/occ_grid.info.resolution)), dtype=int)
        # print(data)

        self.map = [0]*(occ_grid.info.height * occ_grid.info.width + 1)
        
        self.map = np.zeros((occ_grid.info.height, occ_grid.info.width), dtype=int)
        self.map[50:60, 50:60] = -1
        self.map[10:80, 60:70] = -1
        self.map[70:73, 10:80] = -1

        self.map_list = self.map.T.reshape(-1).tolist()

        occ_grid.data = self.map_list   

        self.occ_grid_pub.publish(occ_grid)
        sleep(1)


        # Begin solution
        self.solution()

    def viz(self, path_pose: PlanningNode):
        pose = Pose()
        pose.position.x = float(path_pose.coordinates[0])
        pose.position.y = float(path_pose.coordinates[1])

        q = quaternion_from_euler(0.0, 0.0, path_pose.yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        self.points_to_publish.poses.append(pose)


        self.pose_array_pub.publish(self.points_to_publish)


    def goal_callback(self, data):
        # Target cooridnates
        self.xt = data.x
        self.yt = data.y

    def pose_callback(self, data):
        # Initial cooridnates
        self.x0 = data.x
        self.y0 = data.y


    def solution(self):
        """ Find a solution using A* """

        global Q, visited_Q     # A* Queue 
        Q, visited_Q = [], []
    
        # trajectory: x, y, yaw, phi, time
        xl, yl, yawl, phil, tl = [self.x0], [self.y0], [self.yaw0], [], [0.0]

        # Map related init
        xlb, xub, ylb, yub = -10, 10, -10, 10       # Adjust these bowndaries

        # Init tree
        first_cost_to_go = self.calculate_cost_to_go(self.x0, self.y0, self.xt, self.yt)
        init_Node = PlanningNode(np.array([self.x0, self.y0]), self.yaw0, None, [], 0, first_cost_to_go)
        Q.append(init_Node)

        while Q != []:

            X = self.get_lowest_cost_node(Q)     # Get the element with lowest cost
            visited_Q.append(X)

            print("Q:",len(Q))


            if self.success(X, np.array([self.xt, self.yt])):
                print("SUCCESS")
                # Chain backwards
                node = X

                while node.parent != None:
                    phil = node.control_signals_from_parent + phil

                    for i in range(len(node.control_signals_from_parent)):
                        tl.append(tl[-1] + DT)

                    
                    stamp = self.get_clock().now().to_msg()

                    self.publish_path(stamp, node.coordinates[0], node.coordinates[1], node.yaw)


                    node = node.parent

                return phil, tl
            

            for phi in [-pi/4, 0, pi/4]:
                x, y, yaw = X.coordinates[0], X.coordinates[1], X.yaw
                
                iter = 100 if phi == 0 else 157 # So that I get 90 degrees turns  

                for i in range(iter):
                    # Take small steps
                    x, y, yaw = self.step(x, y, yaw, phi, dt = DT)

                    if self.collision(x, y, xlb, xub, ylb, yub):
                        break


                    phis = [phi]*iter

                if not self.collision(x, y, xlb, xub, ylb, yub):
                    cost_to_come = X.cost_to_come + 1
                    cost_to_go = self.calculate_cost_to_go(x, y, self.xt, self.yt)

                    X_prim = PlanningNode(np.array([x,y]), yaw, X, phis, cost_to_come, cost_to_go)
                        
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
        

    def collision(self, x, y, xlb, xub, ylb, yub):
        """ Checks if coordinates (x, y) is at an obsticle """

        if x <= xlb or x>= xub or y <= ylb or y >= yub:
            # Check boundaries
            return True
          
        x_grid = int(x / self.resolution)
        y_grid = int(y / self.resolution)

        if self.map[x_grid, y_grid] == -1:
            return True
        else:
            return False


    def success(self, c: PlanningNode, target_coordinates):
        if np.linalg.norm(c.coordinates - target_coordinates) <= WIN_RADIUS:
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
    
    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the robot path and publishes the whole path. """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'grid'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

        self._path_pub.publish(self._path)

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



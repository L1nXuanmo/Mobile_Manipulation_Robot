import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from nav_msgs.msg import Path, OccupancyGrid


from geometry_msgs.msg import Twist, Pose2D, Polygon, PoseStamped, PoseArray, Pose, PointStamped
from visualization_msgs.msg import MarkerArray, Marker

from tf_transformations import quaternion_from_euler
from math import atan2,exp,sqrt
import numpy as np
import time


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        # Subscribe to planned path and odometry path
        #TODO add current pose after implementing the localization
        self.create_subscription(Path, "/planned_path", self.planned_path_callback, 500)
        self.create_subscription(PoseStamped, "/recent_odom", self.odom_path_callback, 10)

        # TODO change the name of the topic later
        self.goal_point_publisher = self.create_publisher(PointStamped,'/camera/object/point',10)
        

        # Init variables
        self.current_pose = PoseStamped()
        self.planned_poses = None
        self.next_pose = None
        self.previous_publish = None


        self.path_follower_timer = self.create_timer(0.2, self.path_follower)

        
    def planned_path_callback(self, data):
        if data.poses != []:
            self.planned_poses = data.poses

            print(len(self.planned_poses))
            print(self.planned_poses[0])
            print(self.planned_poses[-1])
            self.path_follower()
        else:
            self.planned_poses=None

    def odom_path_callback(self, data):
        # if data.poses != []:
        self.current_pose = data
        print(self.current_pose.pose.position.x)
            # print(self.current_pose)

    def path_follower(self):
        if self.planned_poses != None:
            if len(self.planned_poses) != 0:
                self.next_pose = self.planned_poses[0]

                delta_x = self.next_pose.pose.position.x - self.current_pose.pose.position.x
                delta_y = self.next_pose.pose.position.y - self.current_pose.pose.position.y
                distance_to_next_point = np.linalg.norm(np.array([delta_x, delta_y]))
                print(len(self.planned_poses), "path len", distance_to_next_point, "distance to next")

                if distance_to_next_point <= 0.7:
                    print("close now!!!")
                    self.planned_poses.pop(0)
                    self.next_pose = self.planned_poses[0]
                    return

                    # If we are close enough to the next point, remove it from the planned poses


                self.next_pose = self.planned_poses[0]
                
                point_to_go = PointStamped()
                point_to_go.point.x =  self.next_pose.pose.position.x
                point_to_go.point.y =  self.next_pose.pose.position.y
                point_to_go.point.z =  0.0
                point_to_go.header.frame_id = self.next_pose.header.frame_id
                
                # Publish the goal point
                if self.next_pose != self.previous_publish:
                    print("published")
                    self.goal_point_publisher.publish(point_to_go)
                    self.previous_publish = self.next_pose


def main(args=None):
    rclpy.init(args=args)

    node = PathFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
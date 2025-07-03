#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, euler_from_quaternion


from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import TransformStamped, PointStamped


class DisplayMarkers(Node):

    def __init__(self) :
        super().__init__('display_markers')

        # Initialize the transform listener and assign it a buffer
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer,self)


        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)


        # Subscribe to aruco marker topic and call callback function on each received message
        self.sub = self.create_subscription(MarkerArray,'/marker_publisher/markers',self.aruco_callback,10)

        self.point_pub = self.create_publisher(
            PointStamped, '/camera/object/point',10
        )
    

    def aruco_callback(self, msg: MarkerArray) :

        # Broadcast/publish the transform between the map frame and the detected aruco marker
        marker = msg.markers[0]
        p = marker.pose.pose
        ps= PointStamped()
        ps.header = marker.header
        ps.point = p.position
        self.point_pub.publish(ps)
        try:
            t = self.tfBuffer.lookup_transform('map','camera_depth_optical_frame', msg.header.stamp)
            p_map = tf2_geometry_msgs.do_transform_pose(p, t)
        except TransformException as ex:
            f'COuld not transform map to camera_color_optical_frame'
            return
        
        

        aruco_t = TransformStamped()
        aruco_t.header.stamp = t.header.stamp
        aruco_t.header.frame_id = 'map'
        aruco_t.child_frame_id = '/aruco/detected'+str(msg.markers[0].id)


        #x,y,z = euler_from_quaternion([t_2.transform.rotation.x,t_2.transform.rotation.y,t_2.transform.rotation.z,t_2.transform.rotation.w])
        x,y,z = euler_from_quaternion([p_map.orientation.x,p_map.orientation.y, p_map.orientation.z, p_map.orientation.w])
        q = quaternion_from_euler(x-math.pi/2,y,z-math.pi/2)
        aruco_t.transform.translation.x= p_map.position.x
        aruco_t.transform.translation.y= p_map.position.y
        aruco_t.transform.translation.z= p_map.position.z
        #aruco_t.transform.rotation = t_2.transform.rotation
        aruco_t.transform.rotation.x=  q[0]
        aruco_t.transform.rotation.y = q[1]
        aruco_t.transform.rotation.z = q[2]
        aruco_t.transform.rotation.w = q[3]

        
        # Send the transformation
        self._tf_broadcaster.sendTransform(aruco_t)



def main() :
    rclpy.init()
    node = DisplayMarkers()
    try :
        rclpy.spin(node)
    except KeyboardInterrupt :
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
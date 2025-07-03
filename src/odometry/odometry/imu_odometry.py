import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from tf2_ros import TransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from geometry_msgs.msg import Twist

import time
from math import e, sqrt, pi, cos, sin

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])


class ImuOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry')

                # initial beliefs
        self.x = 0
        self.y = 0
        self.theta = 0 #This is equivalent to yaw

        #Initialize state variables (also called xk)
        self.mu = np.array([self.x, self.y, self.theta, 0, 0]) 

        #noise in estimate mu, (also known as pk, the prediction error)
        self.sigma_sq = np.array([[0.1, 0, 0, 0, 0],
                                 [0, 0.1, 0, 0, 0],
                                 [0, 0, 0.1, 0, 0],
                                 [0, 0, 0, 0.1, 0],
                                 [0, 0, 0, 0, 0.1]])      

        #Initializing initial odom/imu values
        self.v_odom = 0
        self.w_odom = 0
        self.w_imu = 0

        self.z_t = np.array([self.v_odom, self.w_odom, self.w_imu]) 

        # update step noise (also known as Q)
        # self.sigma_m_sq = rospy.get_param('~sigma_m_sq', 0.01)
        self.sigma_m_sq = np.array([[0.1, 0, 0, 0, 0],
                                   [0, 0.1, 0, 0, 0],
                                   [0, 0, 0.1, 0, 0],
                                   [0, 0, 0, 0.1, 0],
                                   [0, 0, 0, 0, 0.1]])
        # sensor noise
        # self.sigma_z_sq = rospy.get_param('~sigma_z_sq', .1)
        self.sigma_z_sq = np.array([[0.1, 0, 0], #noise in v_odom
                                    [0, 0.3, 0], #noise in w_odom
                                    [0, 0, 0.02]]) #noise in w_gyro
        
        #x_0, y_0, theta_0, v_0, w_0
        self.H = np.array([[0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 1],
                          [0, 0, 0, 0, -1]])

        # Set up publisher to publish to combined Odom
        self.odom_pub = self.create_publisher(Odometry, '/odom_imu', 10)
        self.odomBroadcaster = TransformBroadcaster(self)

        # Set up substriber to imu and odom
        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Twist, '/wheel_odom', self.odom_callback, 10)

        self.dt = 0.2
        self.runner = self.create_timer(self.dt, self.run)
        # self.last_time = self.get_clock().now().seconds_nanoseconds()[1]
        self.last_time = 0

        self.gyro_offset = None

    def imu_callback(self,msg):
        # imu_pose = convert_pose_to_xy_and_theta(msg)
        if self.gyro_offset == None:
            self.gyro_offset = msg.angular_velocity.z
        self.w_imu = msg.angular_velocity.z-self.gyro_offset
        print("IMU_cb", self.w_imu)

    def odom_callback(self,msg):
        # cur_pos = convert_pose_to_xy_and_theta(msg.pose.pose)
        # self.odom_roll = cur_pos[0]
     #    self.odom_pitch = cur_pos[1]
        # self.odom_yaw = cur_pos[2]
        #print 'getting odom values'
        self.w_odom = msg.angular.z
        self.v_odom = msg.linear.x 
        print("odom_cb", self.w_odom, self.v_odom)
        # print "V_ODOM: " + str(self.v_odom)

    def run(self):
        # r = rospy.Rate(20)
        # curr_time = self.get_clock().now()
        # curr_time_int = rclpy.time.Time().nanoseconds
        # print(curr_time_int)
        # curr_time_int = curr_time.seconds_nanoseconds()[1]
        # print(curr_time.seconds_nanoseconds())
        # time.sleep(1)
        # curr_time = self.get_clock().now()

        # print(curr_time.seconds_nanoseconds())

        # last_time = curr_time
        # odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link_imu')
        odom = Odometry()
        # while not rospy.is_shutdown():
            # Do Kalman updates

        # dt = curr_time_int - self.last_time
        # self.last_time = curr_time_int
        print(self.mu, "mu", self.last_time)

        x_k, y_k, theta_k, v_k, w_k = self.mu
        # print self.z_t
        self.z_t = np.array([self.v_odom, self.w_odom, self.w_imu]) 
        #Predict step
        self.mu = np.array([x_k + v_k*self.dt*cos(theta_k), #update state variables
                            y_k + v_k*self.dt*sin(theta_k),
                                theta_k + w_k*self.dt,
                                        v_k,
                                        w_k])
        #The Jacobian of update model
        F_k = np.array([[1, 0, -v_k*self.dt*sin(theta_k), self.dt*cos(theta_k), 0],
                        [0, 1, -v_k*self.dt*cos(theta_k), self.dt*sin(theta_k), 0],
                        [0, 0, 1, 0, self.dt],
                        [0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 1]])
        #update error in prediction
        self.sigma_sq = F_k.dot(self.sigma_sq).dot(F_k.T) + self.sigma_m_sq

        #Update step
        measurement_residual = self.z_t - self.H.dot(self.mu)

        residual_covariance = self.H.dot(self.sigma_sq).dot(self.H.T) + self.sigma_z_sq

        K_t = self.sigma_sq.dot(self.H.T).dot(np.linalg.inv(residual_covariance)) #Kalman gain

        self.mu = self.mu + K_t.dot(measurement_residual)

        self.sigma_sq = (np.eye(len(self.mu))-K_t.dot(self.H)).dot(self.sigma_sq)

        #Publish the new odom message based on the integrated odom values
        # odom.header.stamp = curr_time.to_msg()
        odom.pose.pose.position.x = self.mu[0]
        odom.pose.pose.position.y = self.mu[1]
        odom.pose.pose.position.z = 0.0


        qt_array = quaternion_from_euler(0,0,self.mu[2])
        quaternion = Quaternion(x=qt_array[0], y=qt_array[1], z=qt_array[2], w=qt_array[3])
        quaternion.z = sin(self.mu[2]/2.0)
        quaternion.w = cos(self.mu[2]/2.0)
        odom.pose.pose.orientation = quaternion

        odom.pose.covariance = [self.sigma_sq[0,0], 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in x
                                0.0, self.sigma_sq[1,1], 0.0, 0.0, 0.0, 0.0, #uncertainty in y
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in z
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in roll
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in pitch
                                0.0, 0.0, 0.0, 0.0, 0.0, self.sigma_sq[2,2]] #uncertainty in yaw

        #The velocities are in child frame base_link
        odom.twist.twist.linear.x = self.mu[3]
        odom.twist.twist.angular.z = self.mu[4] 

        odom.twist.covariance = [self.sigma_sq[3,3], 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in x_dot
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in y_dot
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in z_dot
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in change in roll
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, #uncertainty in change in pitch
                                0.0, 0.0, 0.0, 0.0, 0.0, self.sigma_sq[4,4]] #uncertainty in change in yaw

        t = TransformStamped()
        # t.header.stamp = curr_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link_imu'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = self.mu[0]
        t.transform.translation.y = self.mu[1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = quaternion.x
        t.transform.rotation.y = quaternion.y
        t.transform.rotation.z = quaternion.z
        t.transform.rotation.w = quaternion.w

        self.odomBroadcaster.sendTransform(t)
        # self.odom_pub.publish(odom)

        # print(self.mu, "mu", self.last_time)

        # try:
        #     r.sleep()
        # except rospy.exceptions.ROSTimeMovedBackwardsException:
        #     print("Time went backwards. Carry on.")

def main(args=None):

    rclpy.init()
    node = ImuOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
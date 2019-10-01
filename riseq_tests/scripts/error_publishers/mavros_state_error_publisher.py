#!/usr/bin/env python

# support libraries
import rospy
import message_filters
import tf
import numpy as np


# messages
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseStamped, TwistStamped
from riseq_trajectory.msg import riseq_uav_trajectory


g = 9.81

class State_Error_Publisher():

    # constructor
    def __init__(self):

        # publisher 
        self.error_publisher = rospy.Publisher('/riseq/state_error_publisher', Odometry, queue_size = 10)

        self.state = Odometry()
        self.state.pose.pose.orientation.x = 0.0
        self.state.pose.pose.orientation.y = 0.0
        self.state.pose.pose.orientation.z = 0.0
        self.state.pose.pose.orientation.w = 1.0
        self.traj = riseq_uav_trajectory()
        self.traj.rot = [1,0,0,0,1,0,0,0,1]
        self.received_first = False

        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_cb)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velocity_cb)
        self.reftraj_sub = rospy.Subscriber('riseq/trajectory/uav_trajectory', riseq_uav_trajectory, self.traj_cb)

        self.publish_timer = rospy.Timer(rospy.Duration(0.01), self.publish_error)


    def publish_error(self, timer):

        if self.received_first:

            state = self.state
            trajectory = self.traj

            # Extract orientation reference
            ori_quat_ref = [trajectory.pose.orientation.x, trajectory.pose.orientation.y, trajectory.pose.orientation.z, trajectory.pose.orientation.w]
            psi_ref, theta_ref, phi_ref = tf.transformations.euler_from_quaternion(ori_quat_ref, axes = 'rzyx')
            Rbw_ref = np.array([[trajectory.rot[0],trajectory.rot[1],trajectory.rot[2]],         # world to body reference transformation
                             [trajectory.rot[3],trajectory.rot[4],trajectory.rot[5]],
                             [trajectory.rot[6],trajectory.rot[7],trajectory.rot[8]]])
            angular_velocity_ref = np.array([[trajectory.twist.angular.x], [trajectory.twist.angular.y], [trajectory.twist.angular.z]])

            # Extract real orientation
            ori_quat = [state.pose.pose.orientation.x, state.pose.pose.orientation.y, state.pose.pose.orientation.z, state.pose.pose.orientation.w]
            psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes = 'rzyx')
            Rwb = tf.transformations.quaternion_matrix(ori_quat)         # this is an homogenous world to body transformation
            Rbw = Rwb[0:3,0:3].T    # body to world transformation, only rotation part
            angular_velocity = np.array([[state.twist.twist.angular.x],[state.twist.twist.angular.y],[state.twist.twist.angular.z]])

            # extract reference values
            p_ref = np.array([[trajectory.pose.position.x], [trajectory.pose.position.y], [trajectory.pose.position.z]])
            v_ref = np.array([[trajectory.twist.linear.x], [trajectory.twist.linear.y], [trajectory.twist.linear.z]])
            a_ref = np.array([trajectory.acc.x, trajectory.acc.y, trajectory.acc.z]).reshape(3,1)
            euler_dot_ref = np.array([[trajectory.uc.x], [trajectory.uc.y],[trajectory.uc.z]])

            # extract real values
            p = np.array([[state.pose.pose.position.x], [state.pose.pose.position.y], [state.pose.pose.position.z]])
            v = np.array([[state.twist.twist.linear.x], [state.twist.twist.linear.y], [state.twist.twist.linear.z]])
            #v = np.dot(Rbw, v)

            """
            # extract reference values
            x_r, y_r, z_r = [traj_msg.pose.position.x, traj_msg.pose.position.y, traj_msg.pose.position.z]
            vx_r, vy_r, vz_r = [traj_msg.twist.linear.x, traj_msg.twist.linear.y, traj_msg.twist.linear.z] 
            ori_quat_r = [traj_msg.pose.orientation.x, traj_msg.pose.orientation.y, traj_msg.pose.orientation.z, traj_msg.pose.orientation.w]
            psi_r, theta_r, phi_r = tf.transformations.euler_from_quaternion(ori_quat_r, axes = 'rzyx')
            p_r, q_r, r_r = [traj_msg.twist.angular.x, traj_msg.twist.angular.y, traj_msg.twist.angular.z]

            # extract drone real state values
            x, y, z = [state_msg.pose.position.x, state_msg.pose.position.y, state_msg.pose.position.z]
            vx, vy, vz = [state_msg.twist.linear.x, state_msg.twist.linear.y, state_msg.twist.linear.z]
            ori_quat = [state_msg.pose.orientation.x, state_msg.pose.orientation.y, state_msg.pose.orientation.z, state_msg.pose.orientation.w]
            psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes = 'rzyx')
            p, q, r = [state_msg.twist.angular.x, state_msg.twist.angular.y, state_msg.twist.angular.z]
            """

            error_msg = Odometry()
            error_msg.header.stamp = rospy.Time.now()
            error_msg.header.frame_id = 'map'

            error_msg.pose.pose.position.x = p[0] - p_ref[0]
            error_msg.pose.pose.position.y = p[1] - p_ref[1]
            error_msg.pose.pose.position.z = p[2] - p_ref[2]

            error_msg.pose.pose.orientation.x = ori_quat[0] - ori_quat_ref[0]
            error_msg.pose.pose.orientation.y = ori_quat[1] - ori_quat_ref[1]
            error_msg.pose.pose.orientation.z = ori_quat[2] - ori_quat_ref[2]
            error_msg.pose.pose.orientation.w = ori_quat[3] - ori_quat_ref[3]
            error_msg.twist.twist.linear.x = v[0] - v_ref[0]
            error_msg.twist.twist.linear.y = v[1] - v_ref[1]
            error_msg.twist.twist.linear.z = v[2] - v_ref[2]

            error_msg.twist.twist.angular.x = angular_velocity[0] - angular_velocity_ref[0]
            error_msg.twist.twist.angular.y = angular_velocity[1] - angular_velocity_ref[1]
            error_msg.twist.twist.angular.z = angular_velocity[2] - angular_velocity_ref[2]

            self.error_publisher.publish(error_msg)
            #rospy.loginfo(error_msg)

    def position_cb(self, pos):

        self.state.pose.pose.position.x = pos.pose.position.x
        self.state.pose.pose.position.y = pos.pose.position.y
        self.state.pose.pose.position.z = pos.pose.position.z

        self.state.pose.pose.orientation.x = pos.pose.orientation.x
        self.state.pose.pose.orientation.y = pos.pose.orientation.y
        self.state.pose.pose.orientation.z = pos.pose.orientation.z
        self.state.pose.pose.orientation.w = pos.pose.orientation.w

    def velocity_cb(self, vel):

        self.state.twist.twist.linear.x = vel.twist.linear.x
        self.state.twist.twist.linear.y = vel.twist.linear.y
        self.state.twist.twist.linear.z = vel.twist.linear.z

        self.state.twist.twist.angular.x = vel.twist.angular.x
        self.state.twist.twist.angular.y = vel.twist.angular.y
        self.state.twist.twist.angular.z = vel.twist.angular.z
    
    def traj_cb(self, trajectory):
        if not self.received_first:
            self.received_first = True
        self.traj = trajectory


if __name__ == '__main__':
    try:

        rospy.init_node('state_error_publisher', anonymous = True)
        
        state_error_pub = State_Error_Publisher()

        rospy.loginfo(' MAVROS State error Publisher Created !')
        rospy.spin()
        rospy.loginfo(' MAVROS State error Publisher terminated. ')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated.')
        pass
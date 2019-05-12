#!/usr/bin/env python

#ros imports
import rospy
import message_filters
import tf

from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_control.msg import riseq_high_level_control, riseq_low_level_control
from mav_msgs.msg import RateThrust             # for flightgoggles

import control_gains as gains
import numpy as np

class uav_Low_Level_Controller():

    def __init__(self):

        # low level control publisher
        self.llc_pub = rospy.Publisher('riseq/control/uav_low_level_control', riseq_low_level_control, queue_size = 10)

        # flightgoggles publisher 
        self.fg_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size = 10)

        # high level control subscriber
        self.hlc_sub = rospy.Subscriber('riseq/control/uav_high_level_control', riseq_high_level_control, self.feedback_linearization_controller)
        
        # --------------------------------- #
        # Initialize controller parameters  #
        # --------------------------------- #

        # Inertia
        Ixx = rospy.get_param("riseq/Ixx") 
        Iyy = rospy.get_param("riseq/Iyy")
        Izz = rospy.get_param("riseq/Izz")
        self.Inertia = np.diag([Ixx, Iyy, Izz])

        # Thrust-Moment to rotor speed matrix
        kt = rospy.get_param("riseq/thrust_coeff")
        kq = rospy.get_param("riseq/torque_coeff")
        r  = rospy.get_param("riseq/arm_length")
        self.B = np.array([[kt, kt, kt, kt],
                           [0., r*kt, 0, -r*kt],
                           [-r*kt, 0., r*kt, 0.],
                           [-kq, kq, -kq, kq]])
        self.invB = np.linalg.inv(self.B)

        # Gains for euler angle for desired angular velocity
        #       POLE PLACEMENT DESIRED POLES
        # Desired pole locations for pole placement method, for more aggresive tracking
        self.dpr = np.array([-8.0]) 
        self.Kr, self.N_ur, self.N_xr = gains.calculate_pp_gains(gains.Ar, gains.Br, gains.Cr, gains.D_, self.dpr)





    def kai_allibert_control_torque(self, w, w_des, w_dot_ref, gain):
        K_omega = gain
        M = -K_omega*(w - w_des) + np.cross(w,np.dot(params.I,w_des), axis = 0) + np.dot(params.I, w_dot_ref)
        return np.array(M)


    def feedback_linearization_controller(self,hlc):
        """
        @description Calculate control torque M and rotor speeds required to achieve control
        of desired angular speed coming from the high level control message parameter -hlc-
        """

        # extract data
        angular_velocity = np.array([[hlc.angular_velocity.x],[hlc.angular_velocity.y],[hlc.angular_velocity.z]])
        angular_velocity_des = np.array([[hlc.angular_velocity_des.x],[hlc.angular_velocity_des.y],[hlc.angular_velocity_des.z]])
        angular_velocity_dot_ref = np.array([[hlc.angular_velocity_dot_ref.x],[hlc.angular_velocity_dot_ref.y],[hlc.angular_velocity_dot_ref.z]])

        # ------------------------------ #
        #   Control Torque Calculation   #
        # ------------------------------ #
        M = self.feedback_linearization_torque(angular_velocity, angular_velocity_des, angular_velocity_dot_ref, self.Kr.item(0,0))

        # ------------------------------ #
        #   Rotor Speed Calculation      #
        # ------------------------------ #
        T = np.array([[hlc.thrust.z]])
        generalized_input = np.concatenate((T,M),axis=0)    
        w_i = np.dot(self.invB, generalized_input)
        w_i = map(lambda a: np.sqrt(a) if a>0 else -np.sqrt(-a), w_i.flatten())
        print(w_i)
        # ------------------------------ #
        #       Publish message          #
        # ------------------------------ #
        llc_msg = riseq_low_level_control()
        llc_msg.header.stamp = rospy.Time.now()
        llc_msg.header.frame_id = 'riseq/uav'
        llc_msg.thrust.z = hlc.thrust.z
        llc_msg.torque.x = M[0][0]
        llc_msg.torque.y = M[0][0]
        llc_msg.torque.z = M[0][0]
        llc_msg.rotor_speeds = w_i
        self.llc_pub.publish(llc_msg)
        rospy.loginfo(llc_msg)


        # publish to flightgoggles
        fg_msg = RateThrust()
        fg_msg.header.stamp = rospy.Time.now()  
        fg_msg.header.frame_id = 'uav/imu'
        fg_msg.thrust.z = hlc.thrust.z
        fg_msg.angular_rates.x = angular_velocity_des[0][0]
        fg_msg.angular_rates.y = angular_velocity_des[1][0]
        fg_msg.angular_rates.z = angular_velocity_des[2][0]
        self.fg_publisher.publish(fg_msg)

    # definitely need a better name for this
    def feedback_linearization_torque(self, angular_velocity, angular_velocity_des, angular_velocity_dot_ref, gain):
        """
        Based on:
          Mclain, T., Beard, R. W., Mclain, T. ;, Beard, R. W. ;, Leishman, R. C.
          Differential Flatness Based Control of a Rotorcraft For Aggressive Maneuvers 
          (September), 2688-2693.
        """

        # angular velocity error
        w_e = angular_velocity - angular_velocity_des

        # control input ub_e for angular velocity error 
        gain_matrix = np.diag([gain, gain, gain])
        ub_e = -1.0*np.dot(gain_matrix, w_e)

        # control input ub for angular velocity
        ub = ub_e + angular_velocity_dot_ref

        # control torque M
        M = np.dot(self.Inertia, ub) + np.cross(angular_velocity, np.dot(self.Inertia, angular_velocity), axis = 0)

        return M

if __name__ == '__main__':
    try:
        rospy.init_node('riseq_low_level_control', anonymous = True)

        low_level_controller = uav_Low_Level_Controller()

        rospy.loginfo(' Low Level Controller Started! ')
        rospy.spin()
        rospy.loginfo(' High Level Controller Terminated.')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated')
        pass




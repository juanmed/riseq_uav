#!/usr/bin/env python

# Reference papers:
# [1] Faessler, M., Franchi, A., & Scaramuzza, D. (2017). 
#     Differential Flatness of Quadrotor Dynamics Subject to 
#     Rotor Drag for Accurate Tracking of High-Speed Trajectories. 
#     https://doi.org/10.1109/LRA.2017.2776353
# [2] Mellinger, D., & Kumar, V. (2011). 
#     Minimum snap trajectory generation and control for quadrotors. 
#     Proceedings - IEEE International Conference on Robotics and Automation, 2520-2525. 
#     https://doi.org/10.1109/ICRA.2011.5980409
# [3]


#ros imports
import rospy
import message_filters
import tf

from riseq_common.msg import riseq_uav_state
from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_control.msg import riseq_high_level_control

from mav_msgs.msg import RateThrust

    
import riseq_tests.df_flat as df_flat
import control_gains as gains
import numpy as np


class uav_High_Level_Controller():

    def __init__(self):

        # high level control publisher
        self.hlc_pub = rospy.Publisher('riseq/control/uav_high_level_control', riseq_high_level_control, queue_size = 10)

        # flightgoggles publisher 
        self.fg_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size = 10)

        # reference trajectory subscriber
        #self.reftraj_sub = message_filters.Subscriber('riseq/trajectory/uav_reference_trajectory', riseq_uav_trajectory)
        self.reftraj_sub = message_filters.Subscriber('riseq/tests/uav_simple_trajectory', riseq_uav_trajectory)

        # select controller's state input soure: true state, estimated state
        try:
            self.state_input = rospy.get_param('riseq/controller_state_input')
        except:
            print('riseq/controller_state_input parameter unavailable')
            print(' Setting controller state input to: true_state')
            print(' The only possible controller input states  are: true_state, estimated_state, fg_true_state')
            self.state_input = 'true_state'

        if(self.state_input == 'estimated_state'):
            self.state_sub = message_filters.Subscriber('riseq/estimator/uav_estimated_state', riseq_uav_state)

        elif(self.state_input == 'true_state'):
            self.state_sub = message_filters.Subscriber('riseq/tests/uav_ot_true_state', riseq_uav_state)

        elif(self.state_input == 'fg_true_state'):
            # for flight googles simulator
            self.state_sub = message_filters.Subscriber('riseq/tests/uav_fg_true_state', riseq_uav_state)

        else:
            print('riseq/controller_state_input parameter not recognized. Defaulting to true_state')
            print(' The only possible controller input states  are: true_state, estimated_state')
            self.state_sub = message_filters.Subscriber('riseq/tests/uav_ot_true_state', riseq_uav_state)

        # filter messages based on time
        ts = message_filters.ApproximateTimeSynchronizer([self.state_sub, self.reftraj_sub], 10, 0.005) # queue = 10, delay = 0.005s
        
        # select controller's controller type: euler angle based controller, geometric controller
        try:
            self.controller_type = rospy.get_param('riseq/controller_type')
        except:
            print(' riseq/controller_type parameter unavailable')
            print(' Setting controller type to: geometric')
            print(' The only possible controller types are: euler_angle_controller, geometric_controller')
            self.controller_type = 'geometric_controller'

        if( self.controller_type == 'euler_angle_controller'):
            ts.registerCallback(self.euler_angle_controller)

        elif( self.controller_type == 'geometric_controller'):
            ts.registerCallback(self.geometric_controller)

        else:
            print('riseq/controller_type parameter not recognized. Defaulting to geometric_controller')
            print(' The only possible types are: euler_angle_controller, geometric_controller')
            ts.registerCallback(self.geometric_controller)




        # --------------------------------- #
        # Initialize controller parameters  #
        # --------------------------------- #
        
        self.g = rospy.get_param("riseq/gravity")
        self.mass = rospy.get_param("riseq/mass")
        self.thrust_coeff = rospy.get_param("riseq/thrust_coeff")
        self.max_rotor_speed = rospy.get_param("riseq/max_rotor_speed")
        self.rotor_count = rospy.get_param("riseq/rotor_count")
        self.max_thrust = self.rotor_count*self.thrust_coeff*(self.max_rotor_speed**2)  # assuming cuadratic model for rotor thrust 


        self.control_frequency_ratio = 5        # This is the factor by which the position controller is slow
                                                # than orientation controller
        self.pos_error_integral = np.zeros((3,1))       # Store position error integral
        self.Traw = self.g*self.mass                    # Store thrust (before saturation) for anti-windup control

        self.e1 = np.array([[1],[0],[0]])       # Vectors e1, e2, e3 generate R3
        self.e2 = np.array([[0],[1],[0]])
        self.e3 = np.array([[0],[0],[1]])


    def euler_angle_controller(self, state, trajectory):
        """
        @description This controller uses an euler angle representation of orientation
        in order to control it.
        @state quadrotor state
        @trajectory reference trajectory
        """
        if(True):
            # extract reference values
            
            p_ref = np.array([trajectory.pose.position.x, trajectory.pose.position.y, trajectory.pose.position.z]).reshape(3,1)
            v_ref = np.array([trajectory.twist.linear.x, trajectory.twist.linear.y, trajectory.twist.linear.z]).reshape(3,1)
            ori_quat_ref = [trajectory.pose.orientation.x, trajectory.pose.orientation.y, trajectory.pose.orientation.z, trajectory.pose.orientation.w]
            psi_ref, theta_ref, phi_ref = tf.transformations.euler_from_quaternion(ori_quat_ref, axes = 'rzyx')
            # world to body reference transformation
            Rbw_ref = np.array([[trajectory.rot[0],trajectory.rot[1],trajectory.rot[2]],
                             [trajectory.rot[3],trajectory.rot[4],trajectory.rot[5]],
                             [trajectory.rot[6],trajectory.rot[7],trajectory.rot[8]]])

            # extract real values
            p = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z]).reshape(3,1)
            v = np.array([state.twist.linear.x, state.twist.linear.y, state.twist.linear.z]).reshape(3,1)
            ori_quat = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
            # world to body transformation
            Rwb = tf.transformations.quaternion_matrix(ori_quat)
            Rbw = Rwb[0:3,0:3].T    # body to world transformation, only rotation part


            # ---------------------------------------------- #
            #                POSITION CONTROL                #
            #  Compute thrust for trajectory tracking        #
            #  See [1] for details                           #
            # ---------------------------------------------- #

            # PID gains
            Kp = np.diag([gains.Kpx2, gains.Kpy2, gains.Kpz2])
            Kd = np.diag([gains.Kdx2, gains.Kdy2, gains.Kdz2])
            Ki = np.diag([gains.Kix2, gains.Kiy2, gains.Kiz2])

            # Calculate  PID control law for acceleration error: 
            self.pos_error_integral = self.pos_error_integral + (p - p_ref)
            a_e = -1.0*np.dot(Kp,p-p_ref) -1.0*np.dot(Kd,v-v_ref) -1.0*np.dot(Ki,self.pos_error_integral)  # PID control law

            #        ****   Implement anti-windup integral control  ****
            if(self.Traw >= self.max_thrust):
                Ki = np.diag([0.0, 0.0, 0.0])
                a_e2 = -1.0*np.dot(Kp,p-p_ref) -1.0*np.dot(Kd,v-v_ref) -1.0*np.dot(Ki,self.pos_error_integral)  # PID control law
            else:
                a_e2 = a_e            

            # Reference acceleration from differential flatness
            a_ref = np.array([trajectory.acc.x, trajectory.acc.y, trajectory.acc.z]).reshape(3,1)

            # Desired acceleration
            a_des = a_e2 + a_ref + self.g*self.e3

            wzb = np.dot(Rbw, self.e3)          # body z-axis expressed in world frame
            self.Traw = self.mass*np.dot(wzb.T,a_des)[0][0]            # Necessary thrust
            #         ****        Input saturation  *      ****
            T = self.saturate_scalar(self.Traw, self.max_thrust)    # Maximum possible thrust

            # ---------------------------------------------- #
            #             DESIRED ORIENTATION                #
            # Compute desired orientation for trajectory     #
            # tracking                                       #
            # See [1] for details
            # ---------------------------------------------- #

            zb_des = a_des / np.linalg.norm(a_des)

            yc_des = np.array(df_flat.get_yc(psi_ref))
            xb_des = np.cross(yc_des, zb_des, axis = 0)
            xb_des = xb_des/np.linalg.norm(xb_des)
            yb_des = np.cross(zb_des, xb_des, axis = 0)

            #        ****  Discriminate between two possible orientations ****  #
            #   See [2] for Reference
            #   Desired orientation is the one with smallest distance from current
            #   orientation

            # calculate desired orientation
            Rbw_des1 = np.concatenate((xb_des, yb_des, zb_des), axis = 1)
            Rbw_des2 = np.concatenate((-xb_des, -yb_des, zb_des), axis = 1)
            
            angle1 = self.rotation_distance(Rbw, Rbw_des1)
            angle2 = self.rotation_distance(Rbw, Rbw_des2)

            if( angle1 > angle2):
                print("Rbw_des2 selected")
                Rbw_des = Rbw_des2
                xb_des = -xb_des
                yb_des = -yb_des
            else:
                print("Rbw_des1 selected")
                Rbw_des = Rbw_des1

            # Fill out message
            hlc_msg = riseq_high_level_control()
            hlc_msg.header.stamp = rospy.Time.now()
            hlc_msg.header.frame_id = 'riseq/uav'

            hlc_msg.thrust.z = T
            hlc_msg.rot = Rbw_des.flatten().tolist()
            self.hlc_pub.publish(hlc_msg)

            # publish to flightgoggles...just for testing
            fg_msg = RateThrust()
            fg_msg.header.stamp = rospy.Time.now()  
            fg_msg.header.frame_id = 'uav/imu'
            fg_msg.thrust.z = T

            self.fg_publisher.publish(fg_msg)

        else:
            print('hola')

        return 0


    def geometric_controller(self, state, trajectory):
        """
        @description This controller uses an euler angle representation of orientation
        in order to control it.
        @state quadrotor state
        @trajectory reference trajectory
        """

        return 0

    def saturate_scalar(self, value, max_value):
        return min(value,max_value)

    def rotation_distance(self, R1, R2):
        """
        @description Measure the angle to rotate from R1 to R2
        at http://www.boris-belousov.net/2016/12/01/quat-dist/
        @R1 3x3 np.array, must be a member of the Special Orthogonal group SO(3)
        @R2 3x3 np.array, must be a member of the Special Orthogonal group SO(3)
        @return absolute value of the angle between R1 and R2
        """
        Rd = np.dot(R1.T, R2)       # get difference rotation matrix
        angle = np.arccos((np.trace(Rd) -1.0 )/2.0)
        return np.abs(angle)


    def publish_thrust(self, thrust):
        """
        @description publish 
        """
        # create single message
        thrust_msg = riseq_high_level_control()
        thrust_msg.thrust.z = thrust
        self.hlc_pub.publish(thrust_msg)
        rospy.loginfo(thrust_msg)
        rospy.loginfo("Published body vertical thrust: {}".format(thrust))

if __name__ == '__main__':
    try:
        rospy.init_node('uav_input_publisher', anonymous = True)

        # Wait some time before running. This is to adapt to some simulators
        # which require some 'settling time'
        try:
            wait_time = int(rospy.get_param('riseq/high_level_control_wait_time'))
        except:
            print('riseq/high_level_control_wait_time parameter is unavailable')
            print('Setting a wait time of 0 seconds.')
            wait_time = 1

        while( rospy.Time.now().to_sec() < wait_time ):
            if( ( int(rospy.Time.now().to_sec()) % 1) == 0 ):
                rospy.loginfo("Starting Controller in {:.2f} seconds".format(wait_time - rospy.Time.now().to_sec()))
        

        high_level_controller = uav_High_Level_Controller()

        # set to True if using flightgoogles simulator. 
        # This will send some thrust commands to the simulator in order to 'wake up' the IMU
        flightgoggles = True

        if(flightgoggles):
            rate = rospy.Rate(100)
            for i in range(10):
                high_level_controller.publish_thrust(9.9) 
                rate.sleep()        

        rospy.loginfo(' High Level Controller Started! ')
        rospy.spin()
        rospy.loginfo(' High Level Controller Terminated ')    


    except rospy.ROSInterruptException:
     rospy.loginfo('ROS Terminated')
     pass
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
# [3] Kai, J. M., Allibert, G., Hua, M. D., & Hamel, T. (2017). 
#     Nonlinear feedback control of Quadrotors exploiting First-Order Drag Effects. 
#     IFAC-PapersOnLine, 50(1), 8189-8195. https://doi.org/10.1016/j.ifacol.2017.08.1267
# [4] Lee, T., Leok, M., & McClamroch, N. H. (2010). 
#     Control of Complex Maneuvers for a Quadrotor UAV using 
#     Geometric Methods on SE(3) {1}, (i). https://doi.org/10.1002/asjc.0000
 

#ros imports
import rospy
import message_filters
import tf

from riseq_common.msg import riseq_uav_state
from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_control.msg import riseq_high_level_control

# for flightgoggles
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


        self.position_control_frequency_ratio = 10       # This is the factor by which the high_level_controller is slower
                                                         # than low_level controller
        self.position_control_loops = 0                                        
        self.pos_error_integral = np.zeros((3,1))       # Store position error integral
        self.Traw = self.g*self.mass                    # Store thrust (before saturation) for anti-windup control
        self.T = self.g*self.mass
        self.Rbw_des = np.zeros((3,3))
        self.e1 = np.array([[1],[0],[0]])       # Vectors e1, e2, e3 generate R3
        self.e2 = np.array([[0],[1],[0]])
        self.e3 = np.array([[0],[0],[1]])


        # PID gains for position error controller 
        self.Kp = np.diag([gains.Kpx2, gains.Kpy2, gains.Kpz2])
        self.Kd = np.diag([gains.Kdx2, gains.Kdy2, gains.Kdz2])
        self.Ki = np.diag([gains.Kix2, gains.Kiy2, gains.Kiz2])

        # Gains for euler angle for desired angular velocity
        #       POLE PLACEMENT DESIRED POLES
        # Desired pole locations for pole placement method, for more aggresive tracking
        self.dpr = np.array([-8.0]) 
        self.Kr, self.N_ur, self.N_xr = gains.calculate_pp_gains(gains.Ar, gains.Br, gains.Cr, gains.D_, self.dpr)

    def euler_angle_controller(self, state, trajectory):
        """
        @description This controller uses an euler angle representation of orientation
        in order to control it.
        @state quadrotor state
        @trajectory reference trajectory
        """

        # Extract orientation reference
        ori_quat_ref = [trajectory.pose.orientation.x, trajectory.pose.orientation.y, trajectory.pose.orientation.z, trajectory.pose.orientation.w]
        psi_ref, theta_ref, phi_ref = tf.transformations.euler_from_quaternion(ori_quat_ref, axes = 'rzyx')
        Rbw_ref = np.array([[trajectory.rot[0],trajectory.rot[1],trajectory.rot[2]],         # world to body reference transformation
                         [trajectory.rot[3],trajectory.rot[4],trajectory.rot[5]],
                         [trajectory.rot[6],trajectory.rot[7],trajectory.rot[8]]])

        # Extract real orientation
        ori_quat = [state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w]
        psi, theta, phi = tf.transformations.euler_from_quaternion(ori_quat, axes = 'rzyx')
        Rwb = tf.transformations.quaternion_matrix(ori_quat)         # this is an homogenous world to body transformation
        Rbw = Rwb[0:3,0:3].T    # body to world transformation, only rotation part

        # ------------------------------------ #
        #  Thrust and Orientation Computation  #
        # ------------------------------------ #
        if(self.position_control_loops == 0):   # Update desired thrust and desired orientation

            # extract reference values
            p_ref = np.array([trajectory.pose.position.x, trajectory.pose.position.y, trajectory.pose.position.z]).reshape(3,1)
            v_ref = np.array([trajectory.twist.linear.x, trajectory.twist.linear.y, trajectory.twist.linear.z]).reshape(3,1)

            # extract real values
            p = np.array([state.pose.position.x, state.pose.position.y, state.pose.position.z]).reshape(3,1)
            v = np.array([state.twist.linear.x, state.twist.linear.y, state.twist.linear.z]).reshape(3,1)

            # ---------------------------------------------- #
            #                POSITION CONTROL                #
            #  Compute thrust for trajectory tracking        #
            #  See [1] for details                           #
            # ---------------------------------------------- #

            # Calculate  PID control law for acceleration error: 
            self.pos_error_integral = self.pos_error_integral + (p - p_ref)
            a_e = -1.0*np.dot(self.Kp,p-p_ref) -1.0*np.dot(self.Kd,v-v_ref) -1.0*np.dot(self.Ki,self.pos_error_integral)  # PID control law

            #        ****   Implement anti-windup integral control  ****
            if(self.Traw >= self.max_thrust):
                Ki = np.diag([0.0, 0.0, 0.0])
                a_e2 = -1.0*np.dot(self.Kp,p-p_ref) -1.0*np.dot(self.Kd,v-v_ref) -1.0*np.dot(self.Ki,self.pos_error_integral)  # PID control law
            else:
                a_e2 = a_e            

            # Reference acceleration from differential flatness
            a_ref = np.array([trajectory.acc.x, trajectory.acc.y, trajectory.acc.z]).reshape(3,1)

            # Desired acceleration
            a_des = a_e2 + a_ref + self.g*self.e3

            wzb = np.dot(Rbw, self.e3)          # body z-axis expressed in world frame
            self.Traw = self.mass*np.dot(wzb.T,a_des)[0][0]            # Necessary thrust
            #         ****        Input saturation  *      ****
            self.T = self.saturate_scalar(self.Traw, self.max_thrust)    # Maximum possible thrust

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
            #   THIS MUST NOT BE DONE FOR EULER ANGLE BASED CONTROLLER SINCE 
            #   THE REFERENCE YAW (PSI) WILL BE DIRECTLY TRACKED!

            # calculate desired orientation
            self.Rbw_des = np.concatenate((xb_des, yb_des, zb_des), axis = 1)

            """
            Rbw_des2 = np.concatenate((-xb_des, -yb_des, zb_des), axis = 1)
            
            angle1 = self.rotation_distance(Rbw, Rbw_des1)
            angle2 = self.rotation_distance(Rbw, Rbw_des2)
            
            if(angle1 > angle2):
                print("Rbw_des2 selected")
                self.Rbw_des = Rbw_des2
                xb_des = -xb_des
                yb_des = -yb_des
            else:
                print("Rbw_des1 selected")
                self.Rbw_des = Rbw_des1
            """

            self.position_control_loops = self.position_control_loops + 1
        else:
            self.position_control_loops = self.position_control_loops + 1
            if(self.position_control_loops == self.position_control_frequency_ratio):
                self.position_control_loops = 0
            

        # While thrust and desired orientation are not recalculated, send the same previous values.
        # This makes a time separation between the position and the orientation
        # controller, in order for the orientation controller to converge much faster than the 
        # position controller. See [3], [4].

        # ------------------------------------ #
        #     Angular Velocity Computation     #
        # ------------------------------------ #
        euler = np.array([[phi],[theta],[psi]])
        #euler_ref = np.array([[phi_ref],[theta_ref],[psi_ref]])
        euler_des = np.array(df_flat.RotToRPY_ZYX(self.Rbw_des))  # get desired roll, pitch, yaw angles
        euler_dot_ref = np.array([[trajectory.uc.x], [trajectory.uc.y],[trajectory.uc.z]])
        w_des = self.euler_angular_velocity_des(euler, euler_des, euler_dot_ref, self.Kr.item(0,0))

        # Fill out message
        hlc_msg = riseq_high_level_control()
        hlc_msg.header.stamp = rospy.Time.now()
        hlc_msg.header.frame_id = 'riseq/uav'

        hlc_msg.thrust.z = self.T
        hlc_msg.rot = self.Rbw_des.flatten().tolist()
        hlc_msg.angular_velocity.x = w_des[0][0]
        hlc_msg.angular_velocity.y = w_des[1][0]
        hlc_msg.angular_velocity.z = w_des[2][0]
        self.hlc_pub.publish(hlc_msg)
        rospy.loginfo(hlc_msg)

        # publish to flightgoggles...just for testing
        fg_msg = RateThrust()
        fg_msg.header.stamp = rospy.Time.now()  
        fg_msg.header.frame_id = 'uav/imu'
        fg_msg.thrust.z = self.T
        fg_msg.angular_rates.x = w_des[0][0]
        fg_msg.angular_rates.y = w_des[1][0]
        fg_msg.angular_rates.z = w_des[2][0]
        self.fg_publisher.publish(fg_msg)


    def euler_angular_velocity_des(self, euler, euler_ref, euler_dot_ref,gain):
        """
        Control law is of the form: u = K*(euler_ref - euler)
        """
        gain_matrix = np.diag([gain, gain, gain])
        euler_error = euler - euler_ref
        u = -1.0*np.dot(gain_matrix, euler_error)
        
        euler_dot = u + euler_dot_ref

        # compute w_b angular velocity commands as
        #  w_b = K.inv * uc
        #  where  (euler dot) = K*(angular_velocity)
        #  K is -not- a gain matrix, see definition below
        phi = euler[0][0]
        theta = euler[1][0]
        psi = euler[2][0]
        K = np.array([[1.0, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                      [0.0, np.cos(phi), -1.0*np.sin(phi)], 
                      [0.0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

        Kinv = np.linalg.inv(K)        

        w_des = np.dot(Kinv, euler_dot)

        return w_des

    def pucci_angular_velocity_des(self, Rbw, Rbw_des, Rbw_ref_dot, w_ref):
        """
        Calculation of desired angular velocity. See:

        Pucci, D., Hamel, T., Morin, P., & Samson, C. (2014). 
        Nonlinear Feedback Control of Axisymmetric Aerial Vehicles
        https://arxiv.org/pdf/1403.5290.pdf

        Kai, J. M., Allibert, G., Hua, M. D., & Hamel, T. (2017). 
        Nonlinear feedback control of Quadrotors exploiting First-Order Drag Effects. 
        IFAC-PapersOnLine, 50(1), 8189-8195. https://doi.org/10.1016/j.ifacol.2017.08.1267
        """

        # Thrust direction is the body Z axis
        e3 = np.array([[0.0],[0.0],[1.0]])

        # extract real and desired body Z axis
        zb = np.dot(Rbw,e3)
        zb_r = np.dot(Rbw_des,e3)
        zb_r_dot = np.dot(Rbw_ref_dot, e3)

        # Calculation of desired angular velocity is done by 3 terms: an error (or feedback) term,
        # a feed-forward term and a term for the free degree of freedom of rotation around Z axis (Yaw)

        # Feedback term calculation
        k10 = 5.0
        epsilon = 0.01
        k1 = k10  #k10/(1.0 + np.dot(zb.T, zb_r)[0][0] + epsilon)
        lambda_dot = 0.0
        lambda_ = 5.0
        w_fb = (k1 + lambda_dot/lambda_)*np.cross(zb, zb_r, axis= 0)

        # Feed-forward term calculation
        w_ff = 0.0*np.cross(zb_r, zb_r_dot, axis = 0)  #np.array([[0.0],[0.0],[0.0]])

        # Yaw rotation
        w_yaw = 0.0*np.dot(w_ref.T, zb)[0][0] * zb   # np.array([[0.0],[0.0],[0.0]])

        # Convert to body frame
        w_des = np.dot(Rbw.T,w_fb + w_ff + w_yaw)

        return w_des

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
        @description publish thrust values to 'wake up' flightgoggles 
        simulator
        """
        # create single message
        thrust_msg = RateThrust()
        thrust_msg.thrust.z = thrust
        self.fg_publisher.publish(thrust_msg)
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
        # This is important for the simulator to start correctly 
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

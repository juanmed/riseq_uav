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
# [5] Mclain, T., Beard, R. W., Mclain, T. ;, Beard, R. W. ;, Leishman, R. C.
#     Differential Flatness Based Control of a Rotorcraft For Aggressive Maneuvers 
#     (September), 2688-2693. Retrieved 
#     from https://scholarsarchive.byu.edu/facpub%0Ahttps://scholarsarchive.byu.edu/facpub/1949
# [6]  
 

#ros imports
import rospy
import message_filters
import tf

from riseq_common.msg import riseq_uav_state
from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_control.msg import riseq_high_level_control

from mav_msgs.msg import RateThrust             # for flightgoggles
from nav_msgs.msg import Odometry 
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped

import riseq_common.dyn_utils as utils
import numpy as np
#from rpg_controllers import attitude_controller, reinit_attitude_controller
from feedback_linearization_controller import Feedback_Linearization_Controller
from geometric_controller import Geometric_Controller

class uav_High_Level_Controller():

    def __init__(self):
        # determine environment
        environment = rospy.get_param("riseq/environment")

        # high level control publisher
        self.hlc_pub = rospy.Publisher('riseq/control/uav_high_level_control', riseq_high_level_control, queue_size = 10)
        self.px4_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size = 10)

        # flightgoggles publisher
        if(environment == "simulator"): 
            self.fg_publisher = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size = 10)
        else:
            pass

        # reference trajectory subscriber
        #self.reftraj_sub = message_filters.Subscriber('riseq/trajectory/uav_reference_trajectory', riseq_uav_trajectory)
        self.reftraj_sub = message_filters.Subscriber('riseq/trajectory/uav_trajectory', riseq_uav_trajectory)

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
            self.state_sub = message_filters.Subscriber('/mavros/local_position/odom', Odometry)
            #self.state_sub = message_filters.Subscriber('/pelican/odometry_sensor1/odometry', Odometry)
        else:
            print('riseq/controller_state_input parameter not recognized. Defaulting to true_state')
            print(' The only possible controller input states  are: true_state, estimated_state')
            self.state_sub = message_filters.Subscriber('riseq/tests/uav_ot_true_state', riseq_uav_state)

        # filter messages based on time
        ts = message_filters.ApproximateTimeSynchronizer([self.state_sub, self.reftraj_sub], 10, 0.01) # queue = 10, delay = 0.005s
        
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
            ts.registerCallback(self.euler_angle_controller)

        # --------------------------------- #
        # Initialize controller parameters  #
        # --------------------------------- #
        
        self.g = rospy.get_param("riseq/gravity")
        self.mass = rospy.get_param("riseq/mass")
        self.thrust_coeff = rospy.get_param("riseq/thrust_coeff")
        self.max_rotor_speed = rospy.get_param("riseq/max_rotor_speed")
        self.rotor_count = rospy.get_param("riseq/rotor_count")
        self.max_thrust = self.rotor_count*self.thrust_coeff*(self.max_rotor_speed**2)  # assuming cuadratic model for rotor thrust 
        self.min_thrust = 0.0
        self.flc = Feedback_Linearization_Controller(mass = self.mass, max_thrust = self.max_thrust, min_thrust = self.min_thrust)
        self.gc = Geometric_Controller(mass = self.mass, max_thrust = self.max_thrust, min_thrust = self.min_thrust)    
        
        
        # PX4 SITL 
        self.mavros_state = State()
        self.mavros_state.connected = False
        self.mavros_state_sub = rospy.Subscriber('mavros/state', State, self.mavros_state_cb)
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.wait_mavros_connection()
        self.last_mavros_request = rospy.Time.now()
        
        self.enable_sim = rospy.get_param('/riseq/enable_sim', False)
        if(self.enable_sim):
            self.send_setpoints()
            self.status_timer = rospy.Timer(rospy.Duration(0.5), self.mavros_status_cb)
        

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
        v = np.dot(Rbw, v)

        state_ = [p,v,np.zeros((3,1)), np.zeros((3,1)), np.zeros((3,1)),Rbw]
        ref_state = [p_ref, v_ref, a_ref, np.zeros((3,1)), np.zeros((3,1)), Rbw_ref, trajectory.yaw, trajectory.yawdot, trajectory.yawddot, euler_dot_ref]

        #self.T, self.Rbw_des, w_des = self.flc.position_controller(state_, ref_state)
        self.T, self.Rbw_des, w_des = self.gc.position_controller(state_, ref_state)

        #w_des = attitude_controller(Rbw, self.Rbw_des)

        # Fill out message
        hlc_msg = riseq_high_level_control()
        hlc_msg.header.stamp = rospy.Time.now()
        hlc_msg.header.frame_id = 'riseq/uav'

        #hlc_msg.thrust.x = self.Traw2 #np.linalg.norm(self.a_e)     #for debugging purposes
        #hlc_msg.thrust.y = self.Traw #np.linalg.norm(self.a_e2)    #for debugging purposes
        hlc_msg.thrust.z = self.T
        hlc_msg.rot = self.Rbw_des.flatten().tolist()
        hlc_msg.angular_velocity.x = angular_velocity[0][0]
        hlc_msg.angular_velocity.y = angular_velocity[1][0]
        hlc_msg.angular_velocity.z = angular_velocity[2][0]
        hlc_msg.angular_velocity_des.x = 0.1*w_des[0][0]
        hlc_msg.angular_velocity_des.y = 0.1*w_des[1][0]
        hlc_msg.angular_velocity_des.z = 0.1*w_des[2][0]
        hlc_msg.angular_velocity_dot_ref.x = trajectory.ub.x
        hlc_msg.angular_velocity_dot_ref.y = trajectory.ub.y
        hlc_msg.angular_velocity_dot_ref.z = trajectory.ub.z
        #self.hlc_pub.publish(hlc_msg)
        #rospy.loginfo(hlc_msg)

        
        px4_msg = AttitudeTarget()
        px4_msg.header.stamp = rospy.Time.now()
        px4_msg.header.frame_id = 'map'
        px4_msg.type_mask = 7 #px4_msg.IGNORE_ATTITUDE
        q = tf.transformations.quaternion_from_matrix(utils.to_homogeneous_transform(self.Rbw_des))
        px4_msg.orientation.x = q[0]
        px4_msg.orientation.y = q[1]
        px4_msg.orientation.z = q[2]
        px4_msg.orientation.w = q[3]
        px4_msg.body_rate.x = 0.01*w_des[0][0]
        px4_msg.body_rate.y = 0.01*w_des[1][0]
        px4_msg.body_rate.z = 0.01*w_des[2][0]
        px4_msg.thrust =  np.min([1.0, 0.0381*self.T])   #0.56
        self.px4_pub.publish(px4_msg)
        

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

    def publish_thrust(self, thrust):
        """
        @description publish thrust values to 'wake up' flightgoggles 
        simulator. It assumes flightgoggles simulator is running
        """
        # create single message
        thrust_msg = RateThrust()
        thrust_msg.thrust.z = thrust
        self.fg_publisher.publish(thrust_msg)
        rospy.loginfo(thrust_msg)
        rospy.loginfo("Published body vertical thrust: {}".format(thrust))

    def mavros_state_cb(self, state_msg):
        self.mavros_state = state_msg

    def mavros_status_cb(self, timer):

        """
        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = "OFFBOARD"
        arm_cmd = CommandBool()
        arm_cmd.value = True

        if(self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_mavros_request > rospy.Duration(5.0))):
            resp1 = self.set_mode_client(0,offb_set_mode.custom_mode)
            if resp1.mode_sent:
                rospy.loginfo("Requested Offboard Enable")
            self.last_mavros_request = rospy.Time.now()
        elif (not self.mavros_state.armed and (rospy.Time.now() - self.last_mavros_request > rospy.Duration(5.0))):
            arm_client_1 = self.arming_client(arm_cmd.value)
            if arm_client_1.success:
                rospy.loginfo("Requested Vehicle Armed")
            self.last_mavros_request = rospy.Time.now() 
        
        armed = self.mavros_state.armed
        mode = self.mavros_state.mode
        rospy.loginfo("Vehicle armed: {}".format(armed))
        rospy.loginfo("Vehicle mode: {}".format(mode))

        
        if( (not armed ) or (mode != 'OFFBOARD')):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 0            
            self.local_pos_pub.publish(pose)
        else:
            self.status_timer.shutdown()
            #pass
        """

        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = "OFFBOARD"
        arm_cmd = CommandBool()
        arm_cmd.value = True

        if(self.mavros_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_mavros_request > rospy.Duration(5.0))):
            resp1 = self.set_mode_client(0,offb_set_mode.custom_mode)
            if resp1.mode_sent:
                rospy.loginfo("Requested Offboard Enable")
            self.last_mavros_request = rospy.Time.now()

        
        armed = self.mavros_state.armed
        mode = self.mavros_state.mode
        rospy.loginfo("Vehicle armed: {}".format(armed))
        rospy.loginfo("Vehicle mode: {}".format(mode))

        
        if( (not armed ) or (mode != 'OFFBOARD')):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 0            
            self.local_pos_pub.publish(pose)
        else:
            self.status_timer.shutdown()
            #pass

    def send_setpoints(self):
        """
        Publish 100 position setpoints before arming.
        This is required for successfull arming
        """
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

        rate = rospy.Rate(20.0)
        for i in range(100):
            self.local_pos_pub.publish(pose)
            rate.sleep()
            #rospy.loginfo(pose)

    def wait_mavros_connection(self):
        i = 0
        rate = rospy.Rate(20)
        while not self.mavros_state.connected:
            print (" >> {} NOT CONNECTED TO MAVROS <<".format(i))
            rate.sleep()
            i  = i + 1
        print(">> {} CONNECTED TO MAVROS! <<".format(i))

if __name__ == '__main__':
    try:
        rospy.init_node('riseq_high_level_control', anonymous = True)

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

        # if simulator is flightgoggles, this step MUST be done
        #if(rospy.get_param("riseq/environment") == "simulator"):
        #    rate = rospy.Rate(100)
        #    for i in range(10):
        #        high_level_controller.publish_thrust(9.9) 
        #        rate.sleep()        

        rospy.loginfo(' High Level Controller Started! ')
        rospy.spin()
        rospy.loginfo(' High Level Controller Terminated ')    


    except rospy.ROSInterruptException:
     rospy.loginfo('ROS Terminated')
     pass

#!/usr/bin/env python

#ros imports
import rospy
import message_filters
import tf

from riseq_common.msg import riseq_uav_state
from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_control.msg import riseq_high_level_control

#import trajectory.df_flat as df_flat
#import control_gains as gains


class uav_High_Level_Controller():

    def __init__(self):

        # high level control publisher
        self.hlc_pub = rospy.Publisher('riseq/control/uav_high_level_control', riseq_high_level_control, queue_size = 10)

        # reference trajectory subscriber
        self.reftraj_sub = message_filters.Subscriber('riseq/trajectory/uav_reference_trajectory', riseq_uav_trajectory)

        # select controller's state input soure: true state, estimated state
        try:
            self.mode = rospy.get_param('riseq/controller_state_input')
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
        

        self.mass = rospy.get_param("riseq/mass")
        self.thrust_coeff = rospy.get_param("riseq/thrust_coeff")
        self.max_rotor_speed = rospy.get_param("riseq/max_rotor_speed")
        self.rotor_count = rospy.get_param("riseq/rotor_count")
        self.max_thrust = self.rotor_count*self.thrust_coeff*(self.max_rotor_speed**2)  # assuming cuadratic model for rotor thrust 

        self.control_frequency_ratio = 5        # This is the factor by which the position controller is slow
                                                # than orientation controller


    def euler_angle_controller(self, state, trajectory):
        """
        @description This controller uses an euler angle representation of orientation
        in order to control it.
        @state quadrotor state
        @trajectory reference trajectory
        """
        # extract reference values
        p_ref = np.array(trajectory.pose.position).reshape(3,1)
        v_ref = np.array(trajectory.twist.linear).reshape(3,1)
        psi_ref, theta_ref, phi_ref = tf.transformations.euler_from_quaternion(trajectory.pose.orientation, axes = 'rzyx')

        Rbw_ref = np.array([[trajectory.rot[0],trajectory.rot[1],trajectory.rot[2]],
                         [trajectory.rot[3],trajectory.rot[4],trajectory.rot[5]],
                         [trajectory.rot[6],trajectory.rot[7],trajectory.rot[8]]])


        return 0


    def geometric_controller(self, state, trajectory):
        """
        @description This controller uses an euler angle representation of orientation
        in order to control it.
        @state quadrotor state
        @trajectory reference trajectory
        """

        return 0

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


    except rospy.ROSInterruptException:
     rospy.loginfo('ROS Terminated')
     pass
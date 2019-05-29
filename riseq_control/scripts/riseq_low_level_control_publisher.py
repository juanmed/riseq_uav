#!/usr/bin/env python

#ros imports
import rospy
import message_filters
import tf
from std_srvs.srv import Empty

from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_control.msg import riseq_high_level_control, riseq_low_level_control
import riseq_tests.utils as utils 

if(rospy.get_param("riseq/environment") == "simulator"):
    from mav_msgs.msg import Actuators             # for flightgoggles
elif(rospy.get_param("riseq/environment") == "embedded_computer"):
    # import Jetson GPIO for communication with PCA9685
    import sys
    sys.path.append('/opt/nvidia/jetson-gpio/lib/python')
    sys.path.append('/opt/nvidia/jetson-gpio/lib/python/Jetson/GPIO')
    sys.path.append('/home/nvidia/repositories/nano_gpio/gpio_env/lib/python2.7/site-packages/periphery/')
    import Jetson.GPIO as GPIO
    from pca9685_driver import Device
else:
    pass

import control_gains as gains
import numpy as np

class uav_Low_Level_Controller():

    def __init__(self):
        # determine environment
        self.environment = rospy.get_param("riseq/environment")

        # low level control publisher
        self.llc_pub = rospy.Publisher('riseq/control/uav_low_level_control', riseq_low_level_control, queue_size = 10)

        # flightgoggles publisher 
        if(self.environment == "simulator"):
            self.fg_publisher = rospy.Publisher('/pelican/command/motor_speed', Actuators, queue_size = 10)
        else:
            pass

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
        self.max_rotor_speed = rospy.get_param("riseq/max_rotor_speed")
        self.max_xy_torque = r*kt*(self.max_rotor_speed**2)
        self.max_z_torque = kq*(self.max_rotor_speed**2)

        # for drones with rotors aligned to +X, +Y, -X, -Y axis
        self.B = np.array([[kt, kt, kt, kt],
                           [0., r*kt, 0, -r*kt],
                           [-r*kt, 0., r*kt, 0.],
                           [-kq, kq, -kq, kq]])
        
        # for drones with rotors located at 45 degrees from +X, +Y, -X, -Y axis
        q = np.sqrt(2.0)/2.0
        #self.B = np.array([[kt,         kt,      kt,      kt],
        #                   [q*r*kt, q*r*kt, -q*r*kt, -q*r*kt],
        #                   [-q*r*kt, q*r*kt, q*r*kt, -q*r*kt],
        #                   [+kq,        -kq,    +kq,     -kq]])

        self.invB = np.linalg.inv(self.B)

        # Gains for euler angle for desired angular velocity
        #       POLE PLACEMENT DESIRED POLES
        # Desired pole locations for pole placement method, for more aggresive tracking
        if(self.environment == "simulator"):
            self.dpr = np.array([-8.0]) 
            self.Kr, self.N_ur, self.N_xr = gains.calculate_pp_gains(gains.Ar, gains.Br, gains.Cr, gains.D_, self.dpr)
            self.Kr = self.Kr.item(0,0)
        elif(self.environment == "embedded_computer"):
            self.Kr = 8.0
        else:
            print("riseq/environment parameter not found. Setting Kr =8.0")
            self.Kr = 8.0

        # --------------------------------- #
        #  Initialize PCA9685 PWM driver    #
        # --------------------------------- #
        if (self.environment == "embedded_computer"):
            self.pwm_device = self.initPCA9685()
        else:
            pass


        # --------------------------------#
        #    Unpause Gazebo               #
        # --------------------------------#
        if (self.environment == "simulator"):
            rospy.wait_for_service('/gazebo/unpause_physics')
            gazebo_unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            rospy.loginfo(">> Gazebo unpaused at {}".format(rospy.Time.now()))
            #gazebo_unpause()

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
        M = self.feedback_linearization_torque(angular_velocity, angular_velocity_des, angular_velocity_dot_ref, self.Kr)

        # saturate control torque. Apply both maximum limits consequtively
        # the small one will be applied
        M = utils.saturate_vector_dg(M,self.max_xy_torque)
        #M = utils.saturate_vector_dg(M,self.max_z_torque)

        # ------------------------------ #
        #   Rotor Speed Calculation      #
        # ------------------------------ #
        T = np.array([[hlc.thrust.z]])
        generalized_input = np.concatenate((T,M),axis=0)    
        w_i = np.dot(self.invB, generalized_input)
        w_i = map(lambda a: np.sqrt(a) if a>0 else -np.sqrt(-a), w_i.flatten())

        # convert to duty cycles for PCA9685 Chip
        if(self.environment == 'embedded_computer'):
            offset = 1.0
            w_i = map(lambda a: utils.saturate_scalar_minmax(a + offset, self.max_rotor_speed + offset, offset), w_i)   # add offset
            self.set_duty_cycles(self.pwm_device ,w_i)
        elif(self.environment == "simulator"):
            w_i = map(lambda a: utils.saturate_scalar_minmax(a, self.max_rotor_speed, 0.0), w_i) 
        else:
            # implement no saturation
            pass   
        #print(w_i)
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
        if (self.environment == 'simulator'):
            fg_msg = Actuators()
            fg_msg.header.stamp = rospy.Time.now()  
            fg_msg.header.frame_id = ''
            #fg_msg.thrust.z = hlc.thrust.z
            #fg_msg.angular_rates.x = angular_velocity_des[0][0]
            #fg_msg.angular_rates.y = angular_velocity_des[1][0]
            #fg_msg.angular_rates.z = angular_velocity_des[2][0]
            fg_msg.angular_velocities = w_i
            self.fg_publisher.publish(fg_msg)
        else:
            pass

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

    def get_duty_cycles(self, w_i):
        """
        @description Given an array of inputs, convert them to duty cycles values
        to be used for PWM control of rotor speeds. This is to be used in the Jetson
        Nano board with PCA9685 PWM Driver connected through I2C port
        The operation performed on each element i of the array is: 

        duty_cycle_i = element_i + 5
        """
        return map(lambda a: a + 5 , w_i)

    def writeToPCA9685(self, w_i):
        return 0

    def set_channel_duty_cycle(self, pwmdev, channel, dt):
        """
        @pwmdev a Device class object already configured
        @channel Channel or PIN number in PCA9685 to configure 0-15
        @dt desired duty cycle
        """
        val = int((dt*4095)//100)
        pwmdev.set_pwm(channel,val)

    def initPCA9685(self):
        """
        @description configure Jetson Nano GPIO for communication with PCA9685
        @return a PCA9685 Device object to communicate with PCA9685 chip
        """
        GPIO.setmode(GPIO.BOARD)
        mode = GPIO.getmode()
        print("Jetson Nano GPIO Mode: {}".format(mode))

        # discover I2C devices
        i2c_devs = Device.get_i2c_bus_numbers()
        print("The following /dev/i2c-* devices were found:\n{}".format(i2c_devs))

        # Create I2C device
        """
        working_devs = list()
        print("Looking out which /dev/i2c-* devices is connected to PCA9685")
        for dev in i2c_devs:
            try:
                pca9685 = Device(0x40,dev)
                # Set duty cycle
                pca9685.set_pwm(5, 2047)

                # set pwm freq
                pca9685.set_pwm_frequency(1000)
                print("Device {} works!".format(dev))
                working_devs.append(dev)
            except:
                print("Device {} does not work.".format(dev))

        # Select any working device, for example, the first one
        print("Configuring PCA9685 connected to /dev/i2c-{} device.".format(working_devs[0]))
        pca9685 = Device(0x40, working_devs[0]) 
        """
        pca9685 = Device(0x40, 1)    # Immediately set a working device, this assumes PCA9685 is connected to I2C channel 1

        # ESC work at 50Hz
        pca9685.set_pwm_frequency(50)

        # Set slow speed duty cycles
        self.set_channel_duty_cycle(pca9685, 0, 5.0)
        self.set_channel_duty_cycle(pca9685, 1, 5.0)
        self.set_channel_duty_cycle(pca9685, 2, 5.0)
        self.set_channel_duty_cycle(pca9685, 3, 5.0)

        # configure rest of channels to 0 duty cycle
        rest = np.arange(4,16,1)
        for channel in rest:
            self.set_channel_duty_cycle(pca9685, channel, 0.0)

        return pca9685

    def set_duty_cycles(self, pwmdev, dts):
        """
        @description Set the duty cycle for 4 PWM Channels in a PCA9685 device
        @pwmdev PCA9685 Device object with a configured frequency 
        @dts array for duty cycles to set. First element is duty cycle for PCA9685 Channel 0,
        2nd element for PCA9685 Channel 1, 3rd element for PCA9685 Channel 2..etc. Up to Channel 15.
        """ 
        for channel, dt in enumerate(dts):
            self.set_channel_duty_cycle(pwmdev, channel, dt)

    def set_rotors_off(self):
        """
        @description set duty cycles for all rotors to minimum (5%)
        """
        self.set_channel_duty_cycle(self.pwm_device, 0, 5.0)
        self.set_channel_duty_cycle(self.pwm_device, 1, 5.0)
        self.set_channel_duty_cycle(self.pwm_device, 2, 5.0)
        self.set_channel_duty_cycle(self.pwm_device, 3, 5.0)

if __name__ == '__main__':
    try:
        rospy.init_node('riseq_low_level_control', anonymous = True)

        low_level_controller = uav_Low_Level_Controller()
        rospy.loginfo(' Low Level Controller Started! ')
        rospy.spin()
        if(low_level_controller.environment == "embedded_computer"):
            low_level_controller.set_rotors_off()
            rospy.loginfo(' Rotors duty cycle set to 5% to finish. ')
        rospy.loginfo(' Low Level Controller Terminated.')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated')
        pass




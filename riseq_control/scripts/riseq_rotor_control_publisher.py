#!/usr/bin/env python

import rospy
from riseq_control.msg import riseq_low_level_control


# This check maybe should not be done... the node should not start if 
# these dependencies are not available

if(rospy.get_param("riseq/environment") == "embedded_computer"):
    # import Jetson GPIO for communication with PCA9685
    import sys
    sys.path.append('/opt/nvidia/jetson-gpio/lib/python')
    sys.path.append('/opt/nvidia/jetson-gpio/lib/python/Jetson/GPIO')
    sys.path.append('/home/nvidia/repositories/nano_gpio/gpio_env/lib/python2.7/site-packages/periphery/')
    import Jetson.GPIO as GPIO
    from pca9685_driver import Device
else:
    pass


class uav_Rotor_Controller():

    def __init__(self):

        # Subscribe to low level controller
        self.llc_sub = rospy.Subscriber("riseq/control/uav_low_level_control", riseq_low_level_control, publish_duty_cycles)

        # --------------------------------- #
        #  Initialize PCA9685 PWM driver    #
        # --------------------------------- #
        if (self.environment == "embedded_computer"):
            self.pwm_device = self.initPCA9685()
        else:
            pass


    def publish_duty_cycles(self, llc):
        self.set_duty_cycles(self.pwm_device, llc.rotor_speeds)

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

    def set_channel_duty_cycle(self, pwmdev, channel, dt):
        """
        @pwmdev a Device class object already configured
        @channel Channel or PIN number in PCA9685 to configure 0-15
        @dt desired duty cycle
        """
        val = int((dt*4095)//100)
        pwmdev.set_pwm(channel,val)

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
        rospy.init_node('riseq_rotor_control', anonymous = True)

        rotor_controller = uav_Rotor_Controller()
        rospy.loginfo(' Rotor Controller Started! ')
        rospy.spin()
        rotor_controller.set_rotors_off()
        rospy.loginfo(' High Level Controller Terminated.')

    except rospy.ROSInterruptException:
        rospy.loginfo('ROS Terminated')
        pass
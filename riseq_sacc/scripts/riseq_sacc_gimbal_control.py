#! /usr/bin/env python

# mininum 4096 * 5% = 204.8
# maximum 4096 * 10% = 409.6
import rospy
import Adafruit_PCA9685
from std_msgs.msg import Float64
def shutdown():
    pwm.set_pwm(0, 1, 303)

def callback(data):   
    pwm_data = int(data.data)
    pwm.set_pwm(0, 1, pwm_data)
    
if __name__ == '__main__':
    pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum = 1)
    pwm.set_pwm_freq(50)
    
    # first tilt
    pwm.set_pwm(0, 1, 240)

    rospy.init_node("rise_sacc_gimbal_control", anonymous = True)
    rospy.Subscriber("/gimbal_control", Float64, callback)

    rospy.on_shutdown(shutdown) 
    rospy.spin()

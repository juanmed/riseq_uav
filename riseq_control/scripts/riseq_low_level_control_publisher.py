#!/usr/bin/env python

#ros imports
import rospy
import message_filters
import tf

from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_control.msg import riseq_high_level_control


# for flightgoggles
from mav_msgs.msg import RateThrust

import numpy as np

class uav_Low_Level_Controller():

	def __init__(self):

		

	def 

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




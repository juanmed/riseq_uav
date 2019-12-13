#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

import numpy as np

class SNAPsensor():

	def __init__(self):

		rospy.Subscriber('/pelican/odometry_sensor1/odometry', Odometry, self.odom_cb)

		self.snap_pub = rospy.Publisher('/riseq/snap', PointStamped,  queue_size = 10)
		self.jerk_pub = rospy.Publisher('/riseq/jerk', PointStamped,  queue_size = 10)
		self.acc_pub = rospy.Publisher('/riseq/acc', PointStamped,  queue_size = 10)

		self.vel_init = np.zeros(3)
		self.acc_init = np.zeros(3)
		self.jerk_init = np.zeros(3)
		self.snap_init = np.zeros(3)

		self.acc = np.zeros(3)
		self.jerk = np.zeros(3)
		self.snap = np.zeros(3)

		self.t1 = rospy.Time.now()
		self.delta = 0.01



	def odom_cb(self, msg):

		self.acc[0] = (msg.twist.twist.linear.x - self.vel_init[0])/self.delta
		self.acc[1] = (msg.twist.twist.linear.y - self.vel_init[1])/self.delta
		self.acc[2] = (msg.twist.twist.linear.z - self.vel_init[2])/self.delta

		self.vel_init[0] = msg.twist.twist.linear.x
		self.vel_init[1] = msg.twist.twist.linear.y
		self.vel_init[2] = msg.twist.twist.linear.z

		self.jerk[0] = (self.acc[0] - self.acc_init[0])/self.delta
		self.jerk[1] = (self.acc[1] - self.acc_init[1])/self.delta
		self.jerk[2] = (self.acc[2] - self.acc_init[2])/self.delta

		self.acc_init[0] = self.acc[0]
		self.acc_init[1] = self.acc[1]
		self.acc_init[2] = self.acc[2]

		self.snap[0] = (self.jerk[0] - self.jerk_init[0])/self.delta
		self.snap[1] = (self.jerk[1] - self.jerk_init[1])/self.delta
		self.snap[2] = (self.jerk[2] - self.jerk_init[2])/self.delta

		self.jerk_init[0] = self.jerk[0]
		self.jerk_init[1] = self.jerk[1]
		self.jerk_init[2] = self.jerk[2]

		snap_msg = PointStamped()
		snap_msg.header.stamp = rospy.Time.now()
		snap_msg.point.x = self.snap[0]
		snap_msg.point.y = self.snap[1]
		snap_msg.point.z = self.snap[2]
		self.snap_pub.publish(snap_msg)

		jerk_msg = PointStamped()
		jerk_msg.header.stamp = rospy.Time.now()
		jerk_msg.point.x = self.jerk[0]
		jerk_msg.point.y = self.jerk[1]
		jerk_msg.point.z = self.jerk[2]
		self.jerk_pub.publish(jerk_msg)

		acc_msg = PointStamped()
		acc_msg.header.stamp = rospy.Time.now()
		acc_msg.point.x = self.acc[0]
		acc_msg.point.y = self.acc[1]
		acc_msg.point.z = self.acc[2]
		self.acc_pub.publish(acc_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('snap_sensor', anonymous = True)

        sensor = SNAPsensor()

        rospy.loginfo(' SNAP Sensor Started! ')
        rospy.spin()
        rospy.loginfo(' SNAP Sensor Terminated ') 

    except rospy.ROSInterruptException:
     rospy.loginfo('ROS Terminated')
     pass 
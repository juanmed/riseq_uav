#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


zedpose = np.zeros(3)
optipose = np.zeros(3)

def zedpose_cb(odom):
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	z = odom.pose.pose.position.z
	zedpose[0] = x
	zedpose[1] = y
	zedpose[2] = z

def optitrack_cb(msg):
	x = msg.pose.position.x
	y = msg.pose.position.y
	z = msg.pose.position.z
	optipose[0] = x
	optipose[1] = y
	optipose[2] = z

def main():

	gatepose_pub = rospy.Publisher("/riseq/perception/gate_pose", PoseStamped, queue_size = 10)
	zedcam_sub = rospy.Subscriber("/zed/zed_node/odom", Odometry, zedpose_cb)
	optitrack_sub = rospy.Subscriber("/vrpn_client_node/Fastquad/pose", PoseStamped, optitrack_cb)
	gate_pose = np.array([3.5, 0.0, 1.21])

	r = rospy.Rate(30)
	while not rospy.is_shutdown():

		gate_wrt_zed = gate_pose - optipose
		#print(gate_wrt_zed)
		gate_msg = PoseStamped()
		gate_msg.header.stamp = rospy.Time.now()
		gate_msg.pose.position.x = gate_wrt_zed[0]
		gate_msg.pose.position.y = gate_wrt_zed[1]
		gate_msg.pose.position.z = gate_wrt_zed[2]
		gatepose_pub.publish(gate_msg)
		r.sleep()


if __name__ == '__main__':
	try:

		# init node
		rospy.init_node('iros_gate_pose_Publisher', anonymous = True)
		main()
		rospy.loginfo('IROS Gate Pose Publisher Started')
		rospy.spin()
		rospy.loginfo('IROS Gate Pose Publisher Terminated')
	except rospy.ROSInterruptException:
		print("ROS Terminated.")
		pass

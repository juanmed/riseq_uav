#!/usr/bin/env python

# ROS packages import
import rospy
import message_filters
import tf

# messages import
from geometry_msgs.msg import PoseStamped
from riseq_common.msg import riseq_uav_state

# other imports
import numpy as np

class fgTrueStatePublisher():

	def __init__(self):
		"""
		Initialize drone state for publishing ground truth of simulated drone in Flightgoggles
		"""

		# create topic uav state publisher 
		self.state_pub = rospy.Publisher("riseq/tests/uav_fg_true_state", riseq_uav_state, queue_size = 10)

		# create message filter
		self.pose_sub = rospy.Subscriber('riseq/tests/uav_fg_true_pose', PoseStamped, self.state_calculation)

		# Initialize pose
		try:
			init_pose = rospy.get_param("/uav/flightgoggles_uav_dynamics/init_pose")
		except:
			print("/uav/flightgoggles_uav_dynamics/init_pose parameter is unavailable")
			print("Initializing pose as:  \n pose = [0,0,0] \norientation = [0,0,0,1]")
			init_pose = [0,0,0,0,0,0,1]
		
		# init position
		self.init_pos = np.zeros(3)
		self.init_pos[0] = init_pose[0]
		self.init_pos[1] = init_pose[1]
		self.init_pos[2] = init_pose[2]

		# init orientation
		self.init_ori = np.zeros(4)
		self.init_ori[0] = init_pose[3]
		self.init_ori[1] = init_pose[4]
		self.init_ori[2] = init_pose[5]
		self.init_ori[3] = init_pose[6]

		# init timestamp
		self.t1 = rospy.Time.now()

	def state_calculation(self, pose_msg):
		"""
		Given the just received pose in pose_msg and the previous pose stored in self.init_pose
		and self.init_ori, calculate both linear and angular velocity, and publish it as 
		ground true state
		"""

		true_state = riseq_uav_state()

		true_state.header.stamp = rospy.Time.now()
		true_state.header.frame_id = ""

		# Fill linear and angular position
		true_state.pose.position.x = pose_msg.pose.position.x
		true_state.pose.position.y = pose_msg.pose.position.y
		true_state.pose.position.z = pose_msg.pose.position.z

		true_state.pose.orientation.x = pose_msg.pose.orientation.x
		true_state.pose.orientation.y = pose_msg.pose.orientation.y
		true_state.pose.orientation.z = pose_msg.pose.orientation.z
		true_state.pose.orientation.w = pose_msg.pose.orientation.w	
		
		# ----------------------------------- #
		#     Calculate Linear velocity		  #
		# use v = dx/dt = (x2 - x1)/(t2 - t1) #
		# ----------------------------------- #

		# time delta dt
		time_delta = pose_msg.header.stamp - self.t1
		time_delta = time_delta.to_sec() 		# convert to floating point
		self.t1 = pose_msg.header.stamp  		# prepare for next time step

		# init velocity
		v = np.zeros(3)
		v[0] = (pose_msg.pose.position.x - self.init_pos[0])/time_delta
		v[1] = (pose_msg.pose.position.y - self.init_pos[1])/time_delta
		v[2] = (pose_msg.pose.position.z - self.init_pos[2])/time_delta			

		# prepare for next time step
		self.init_pos[0] = pose_msg.pose.position.x
		self.init_pos[1] = pose_msg.pose.position.y
		self.init_pos[2] = pose_msg.pose.position.z

		# fill calculated velocity into message
		true_state.twist.linear.x = v[0]
		true_state.twist.linear.y = v[1]
		true_state.twist.linear.z = v[2]

		# ----------------------------------- #
		#     Calculate Angular velocity	  #
		# look: https://math.stackexchange.com/questions/2282938/converting-from-quaternion-to-angular-velocity-then-back-to-quaternion
		# w_b = 2*Im([q(t-h)'*q(t)])/delta_t  q(t)': is the congujate

		# This is q(t)=[v2 w2] current orientation where v2 is the vector part
 		ori = np.zeros(4)
		ori[0] = pose_msg.pose.orientation.x
		ori[1] = pose_msg.pose.orientation.y
		ori[2] = pose_msg.pose.orientation.z
		ori[3] = pose_msg.pose.orientation.w

		# This is q(t-h)=[v1 w1] previous orientation, where v1 is the vector part
		prev_ori_cong = np.zeros(4)	
		prev_ori_cong[0] = -1.0*self.init_ori[0]
		prev_ori_cong[1] = -1.0*self.init_ori[1]
		prev_ori_cong[2] = -1.0*self.init_ori[2]
		prev_ori_cong[3] = self.init_ori[3]

		# Im(q1*q2) = [w1*v2 + w2*v1 + v1xv2]
		w_b = prev_ori_cong[3]*ori[0:3] + ori[3]*prev_ori_cong[0:3] + np.cross(prev_ori_cong[0:3],ori[0:3])
		w_b = 2.0*w_b/time_delta		

		# prepare for next step
		self.init_ori[0] = ori[0]
		self.init_ori[1] = ori[1]
		self.init_ori[2] = ori[2]
		self.init_ori[3] = ori[3]

		# fill message with computed angular velocity
		true_state.twist.angular.x = w_b[0]
		true_state.twist.angular.y = w_b[1]
		true_state.twist.angular.z = w_b[2]

		self.state_pub.publish(true_state)
		rospy.loginfo(true_state)

if __name__ == '__main__':
	try:

		# init node
		rospy.init_node('riseq_fg_true_state_publisher', anonymous = True)

		# create true state publisher for flight goggles simulator
		true_state_pub = fgTrueStatePublisher()

		rospy.loginfo('Flightgoggles True State Publisher Started')
		rospy.spin()
		rospy.loginfo('Flightgoggles True State Publisher Terminated')
	except rospy.ROSInterruptException:
		print("ROS Terminated.")
		pass
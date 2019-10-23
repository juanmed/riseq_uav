#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

markers = ['o', '^', 'x']
colors = ['green', 'yellow', 'r']

class ErrorMap():
    def __init__(self):
        self.gate_down = None
        self.drone_pose = None
        self.distance_computed = None

        rospy.Subscriber("/vrpn_client_node/Fastquad/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/vrpn_client_node/gatedown/pose", PoseStamped, self.gate_down_cb)
        rospy.Subscriber("/riseq/perception/computed_position", PoseStamped, self.gate_computed_cb)

        while self.gate_down is None or self.distance_computed is None:
            print "waiting"
            rospy.sleep(0.1)

        self.rate = 2  # hz

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax.scatter(self.gate_down.pose.position.x, self.gate_down.pose.position.y, self.gate_down.pose.position.z, color="black")

    def gate_down_cb(self, msg):
        self.gate_down = msg

    def pose_cb(self, msg):
        self.drone_pose = msg

    def gate_computed_cb(self, msg):
        self.distance_computed = msg

    def error_map(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            x = self.drone_pose.pose.position.x
            y = self.drone_pose.pose.position.y
            z = self.drone_pose.pose.position.z

            ground_truth = np.zeros(3)
            ground_truth[0] = self.gate_down.pose.position.x - x
            ground_truth[1] = self.gate_down.pose.position.y - y
            ground_truth[2] = self.gate_down.pose.position.z - z

            estimation = np.zeros(3)
            estimation[0] = self.distance_computed.pose.position.x
            estimation[1] = self.distance_computed.pose.position.y
            estimation[2] = self.distance_computed.pose.position.z

            error = ground_truth - estimation
            error_norm = np.linalg.norm(error)

            if error_norm < 0.3:
                self.ax.scatter(x, y, z, c=colors[0], marker=markers[0])
            elif error_norm < 0.6:
                self.ax.scatter(x, y, z, c=colors[1], marker=markers[1])
            else:
                self.ax.scatter(x, y, z, c=colors[2], marker=markers[2])
            print "ploting"
            r.sleep()
        print "plot data"
        plt.show()


if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('iros_errormap', anonymous = True)
        rospy.loginfo('IROS error map Started')
        EM = ErrorMap()
        EM.error_map()

        rospy.loginfo('IROS error map Terminated')

    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass

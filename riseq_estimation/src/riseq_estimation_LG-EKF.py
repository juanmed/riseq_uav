#!/usr/bin/env python

import rospy
import numpy as np


class LG-EKF():
    def __init__(self):
        rospy.init_node('riseq_estimation_LG-EKF')

        self.frequency = 250
        self.r = rospy.Rate(self.frequency)
        self.dt = 1.0/self.frequency
        self.g = 9.81

        self.x_pre = np.array
        self.x_est
        self.P_pre
        self.P_est
        self.z

        self.F
        self.Phi
        self.H


    def loop(self):
        self.predict()
        self.update()

        self.r.sleep()


if __name__ == "__main__":
    lg-ekf = LG-EKF()
    while not rospy.is_shutdown():
        lg-ekf.loop()
#!/usr/bin/env python

import rospy
import numpy as np


class LieGroupExtendedKalmanFilter():
    def __init__(self):
        rospy.init_node('riseq_estimation_LGEKF')

        self.frequency = 250
        self.r = rospy.Rate(self.frequency)
        self.dt = 1.0/self.frequency
        self.g = 9.81

        # state variables: T, w, v
        # SE(3) x R3 x R3
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
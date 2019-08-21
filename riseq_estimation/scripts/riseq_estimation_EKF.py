#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.0.0
brief: This code is using Lie-group Extended Kalman Filter to estimate drones' states.

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import rospy
import numpy as np

import lie


class ExtendedKalmanFilter():
    def __init__(self):
        rospy.init_node('riseq_estimation_EKF')

        self.frequency = 250
        self.r = rospy.Rate(self.frequency)
        self.dt = 1.0/self.frequency
        self.g = 9.81

        # State variables: T, w_b, v_b, scale
        self.x_pre = np.eye((12, 12))
        self.x_est = np.eye((12, 12))

        # Initial VO scale
        self.x_pre[10][11] = 1
        self.x_est[10][11] = 1

        self.P_pre
        self.P_est

        # Measurement: gyro->w_b, accel->a_b, VO->v_b,w_b
        self.z = np.eye((16, 16))

        self.F
        self.Phi
        self.H


    def loop(self):
        self.x_pre = self.predict(self.x_est)
        self.x_est = self.update()

        self.r.sleep()


    def predict(self, x):


    def update(self, x):

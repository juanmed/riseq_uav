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
#!/usr/bin/env python

import rospy
import numpy as np


class IROSGateSearcher:
    def __init__(self, initial_yaw = 0.0):
        self.initial_yaw = initial_yaw
        self.yaw = initial_yaw
        self.turn_left = True
        self.turn_right = False

        self.theta = rospy.get_param("/perception/yaw_size", 30)
        self.searching = False

        self.dilate_iter = 2
        self.erode_iter = 4
        self.dilate_max = 5
        self.erode_min = 1

    def search_gate(self):
        print "search gate"
        if self.turn_left:
            if self.yaw < self.initial_yaw + np.pi/2:
                print "turn left"
                self.yaw = self.yaw + self.theta / 180.0 * np.pi
            else:
                # if drone rotates over 90 degree, it might detect another gate which drone should not pass.
                self.turn_left = False
                self.turn_right = True
                self.yaw = self.initial_yaw

        elif self.turn_right:
            if self.yaw < self.initial_yaw - np.pi/2:
                print "turn right"
                self.yaw = self.yaw - self.theta / 180.0 * np.pi
            else:
                # if drone rotates over 90 degree, it might detect another gate which drone should not pass.
                self.turn_left = False
                self.turn_right = False
                self.yaw = self.initial_yaw

        else:
            if self.erode_iter > self.erode_min:
                print "erode down"
                self.erode_iter = self.erode_iter - 1
            else:
                if self.dilate_iter < self.dilate_max:
                    print "dilate up"
                    self.dilate_iter = self.dilate_iter + 1
                else:
                    print "cannot find gate..."
                    self.dilate_iter = 2
                    self.erode_iter = 4
                    return False, self.yaw  # This means that there is no more step.
            self.turn_left = True
            self.turn_right = False

        return True, self.yaw

#!/usr/bin/env python

import rospy
import numpy as np


class LowPassFilter:
    def __init__(self, cutoff_frequency, rate):
        f = cutoff_frequency
        w = 2 * np.pi * f
        self.tau = 1 / w
        self.ts = rate

        self.last_y = None

    def low_pass_filter(self, current_x):
        """
        Function to calculate low pass filter
        Y(n+1) = (tau * Y(n) + ts * X(n))/(tau + ts)
        :param current_x: X(n)
        :return: Y(n+1)
        """
        if self.last_y is not None:
            self.last_y = (self.tau * self.last_y + self.ts * current_x) / (self.tau + self.ts)
            return self.last_y
        else:
            self.last_y = current_x
            return self.last_y


if __name__ == "__main__":
    rospy.init_node('riseq_gate_lowpassfilter', anonymous=True)

    rospy.spin()

    try:
        rospy.loginfo("Gate lowpass filter created")
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
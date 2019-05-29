#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int32, Float32


class Calculator():
    def data_cb(self, data):
        if self.n < self.max:
            self.average = (self.average * self.n + float(data.data)) / float(self.n + 1)
            print self.average
            self.arr[self.n] = float(data.data)
            self.n += 1

        if self.n == self.max:
            total = 0.0
            for i in range(self.n):
                total += (self.average - self.arr[i])**2
            self.variance = total / self.n


    def __init__(self):
        rospy.init_node('riseq_variance_calculator')

        self.rate = 10
        self.r = rospy.Rate(self.rate)

        self.variance_publisher = rospy.Publisher('/variance', Float32, queue_size=10)

        self.n = 0
        self.max = 10000
        self.arr = np.zeros((self.max))
        self.average = 0.0
        self.variance = 0.0

        rospy.Subscriber('/distance', Int32, self.data_cb)


    def loop(self):
        if self.n >= self.max:
            self.variance_publisher.publish(self.variance)
        self.r.sleep()


if __name__ == "__main__":
    calculator = Calculator()
    while not rospy.is_shutdown():
        calculator.loop()

#!/usr/bin/env python
"""
author:  Eugene Auh
version: version of this script
brief: This code calculates the characteristics of a sensor. Subscribes data and calculates average and covariance of the noise.

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
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu


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


    def imu_cb(self, data):
        if self.n < self.max:
            self.average_x = (self.average_x * self.n + float(data.angular_velocity.x)) / float(self.n + 1)
            self.arr_x[self.n] = float(data.angular_velocity.x)
            self.average_y = (self.average_y * self.n + float(data.angular_velocity.y)) / float(self.n + 1)
            self.arr_y[self.n] = float(data.angular_velocity.y)
            self.average_z = (self.average_z * self.n + float(data.angular_velocity.z)) / float(self.n + 1)
            self.arr_z[self.n] = float(data.angular_velocity.z)
            self.n += 1
            print self.n

        if self.n == self.max:
            total_x = 0.0
            total_y = 0.0
            total_z = 0.0
            for i in range(self.n):
                total_x += (self.average_x - self.arr_x[i])**2
                total_y += (self.average_y - self.arr_y[i])**2
                total_z += (self.average_z - self.arr_z[i])**2
            self.variance_x = total_x / self.n
            self.variance_y = total_y / self.n
            self.variance_z = total_z / self.n

            print self.average_x, self.average_y, self.average_z
            print self.variance_x, self.variance_y, self.variance_z


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
        self.arr_x = np.zeros((self.max))
        self.average_x = 0.0
        self.variance_x = 0.0
        self.arr_y = np.zeros((self.max))
        self.average_y = 0.0
        self.variance_y = 0.0
        self.arr_z = np.zeros((self.max))
        self.average_z = 0.0
        self.variance_z = 0.0

        rospy.Subscriber('/distance', Int32, self.data_cb)
        rospy.Subscriber('/mavros/imu/data_raw', Imu, self.imu_cb)


    def loop(self):
        if self.n >= self.max:
            self.variance_publisher.publish(self.variance)
        self.r.sleep()


if __name__ == "__main__":
    calculator = Calculator()
    while not rospy.is_shutdown():
        calculator.loop()

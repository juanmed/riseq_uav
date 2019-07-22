#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: This node calculates and publishes the linear velocities of a UAV using downward camera, IMU and downward rangefinder.

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
from matplotlib import pyplot as plt
import cv2
import cv_bridge
import message_filters
from std_msgs.msg import Int32
from sensor_msgs.msg import CameraInfo, Image, Imu
from geometry_msgs.msg import Vector3

from eugene_kinematics import pseudoInverseMatrixL


class OpticalFlow():
    def camera_info_cb(self, msg):
        """
        Save the informations of the camera.
        e.g. size of the image, camera matrix
        """
        self.width = msg.width
        self.height = msg.height
        self.camera_matrix = np.array(msg.K).reshape((3, 3))


    def imgCb(self, msg):
        """
        Subscribe image topic, then calculate optical flow by FAST and KLT.
        Publish the linear velocities of the uav on body frame from optical flow, imu and rangefinder
        """
        img_gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        if self.first:
            self.kp0 = self.fast.detect(img_gray, None)
            #self.kp0 = self.orb.detect(img_gray, None)
            #self.kp0, self.des = self.orb.compute(img_gray, self.kp0)
        else:
            
            if len(self.kp1) < 100:
                self.kp0 = self.fasst.detect(img_gray, None)
                #self.kp0 = self.orb.detect(img_gray, None)
                #self.kp0, self.des = self.orb.compute(img_gray, self.kp0)

        img_feature = cv2.drawKeypoints(img_gray, self.kp0, None)
        cv2.imshow('feature', img_feature)
        cv2.waitKey(1)


    def rangefinderCb(self, msg):
        """
        Distance between the uav and a floor measured by downward rangefinder.
        """
        self.distance = msg.data


    def __init__(self):
        rospy.init_node('riseq_estimation_opticalflow')

        self.bridge = cv_bridge.CvBridge()
        self.rate = 50
        self.r = rospy.Rate(self.rate)

        self.velocity = Vector3()
        self.velocity_publisher = rospy.Publisher('/riseq/estimation/of_velocity', Vector3, queue_size=10)
        rospy.Subscriber('/camera/image_raw', Image, self.imgCb)
        #rospy.Subscriber('/riseq/sensor/imu', Imu, self.imuCb)
        rospy.Subscriber('/riseq/sensor/rangefinder', Int32, self.rangefinderCb)

        self.orb = cv2.ORB_create(1000)
        self.fast = cv2.FastFeatureDetector_create(30)
        self.fast.setNonmaxSuppression(True)


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    try:
        print "OpenCV: " + cv2.__version__
        optical_flow = OpticalFlow()
        while not rospy.is_shutdown():
            optical_flow.loop()
    except rospy.ROSInterruptException:
        pass

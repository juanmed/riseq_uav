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
from geometry_msgs.msg import TwistStamped

import lie


class VisualOdometry():
    def imgCb(self, msg):
        """
        Subscribe image topic, then calculate visual odometry FAST and KLT.
        Calculates SE(3) matrix between two image and publish linear, angular velocities.
            linear: unscaled velocity on body frame
            angular: rad/sec on body frame
        """
        self.vo.header.stamp = rospy.Time.now()
        self.img_cur = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        if self.first_frame:
            self.p1 = self.fast.detect(self.img_cur, None)
            self.p1 = np.array([x.pt for x in self.p1], dtype=np.float32)
            self.first_frame = False

        else:
            self.p1, status, err = cv2.calcOpticalFlowPyrLK(self.img_ref, self.img_cur, self.p0, None, **self.lk_params)
            self.p0 = self.p0[status[:, 0]==1]
            self.p1 = self.p1[status[:, 0]==1]

            #print len(self.p1)
            if len(self.p1) < 100:
                self.p1 = self.fast.detect(self.img_cur, None)
                self.p1 = np.array([x.pt for x in self.p1], dtype=np.float32)
            else:
                E, mask = cv2.findEssentialMat(self.p0, self.p1, focal=self.fx, pp=(self.cx, self.cy), method=cv2.RANSAC, prob=0.999, threshold=0.25)
                _, R, t, mask = cv2.recoverPose(E, self.p0, self.p1, focal=self.fx, pp=(self.cx, self.cy))

                T = np.eye(4)
                T[0:3, 0:3] = R
                T[0:3, 3] = t.T
                self.T0 = np.dot(self.T0, T)
                print T

                w = lie.vee3(lie.log_so3(R)) * self.rate
                v = t * self.rate
                self.vo.twist.linear.x = v[0]
                self.vo.twist.linear.y = v[1]
                self.vo.twist.linear.z = v[2]
                self.vo.twist.angular.x = w[0]
                self.vo.twist.angular.y = w[1]
                self.vo.twist.angular.z = w[2]
                self.vo_publisher.publish(self.vo)
                #print self.vo

            # Visualization
            vis = self.img_cur.copy()
            mask = np.zeros_like(self.img_ref)
            mask[:] = 255
            for (x, y) in self.p1:
                cv2.circle(vis, (x, y), 3, (0, 255, 0), -1)
            cv2.imshow('feature', vis)
            cv2.waitKey(10) & 0xFF

        self.img_ref = self.img_cur
        self.p0 = self.p1


    def cameraInfoCb(self, msg):
        if self.fx == 0:
            self.K = np.array(msg.K).reshape((3, 3))
            self.D = np.array(msg.D)[0:4]
            self.fx = self.K[0, 0]
            self.cx = self.K[0, 2]
            self.cy = self.K[1, 2]
        else:
            pass


    def __init__(self):
        rospy.init_node('riseq_estimation_visual_odometry')

        self.T0 = np.eye(4)

        self.bridge = cv_bridge.CvBridge()
        self.rate = 100
        self.r = rospy.Rate(self.rate)

        self.fx = 0
        self.first_frame = True
        self.fast = cv2.FastFeatureDetector_create(threshold=35, nonmaxSuppression=True)
        self.lk_params = dict(winSize = (15, 15), maxLevel = 2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.vo = TwistStamped()
        self.vo_publisher = rospy.Publisher('/riseq/estimation/vo', TwistStamped, queue_size=10)

        rospy.Subscriber('/camera/camera_info', CameraInfo, self.cameraInfoCb)
        #rospy.Subscriber('/riseq/camera/image_rect', Image, self.imgCb)
        rospy.Subscriber('/camera/image_raw', Image, self.imgCb)


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    try:
        #print "OpenCV: " + cv2.__version__
        vo = VisualOdometry()
        while not rospy.is_shutdown():
            vo.loop()
    except rospy.ROSInterruptException:
        pass

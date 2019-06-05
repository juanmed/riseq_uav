#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv
import cv_bridge
import message_filters
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from geometry_msgs.msg import Twist

from eugene_kinematics import pseudoInverseMatrixL


class OpticalFlow():
    def camera_info_cb(self, info):
        """
        Save the informations of the camera.
        e.g. size of the image, camera matrix
        """
        self.width = info.width
        self.height = info.height
        self.camera_matrix = np.array(info.K).reshape((3, 3))


    def getStereoJ(self, n):
        """
        Calculate the relationship between image plane and real space.
        """
        x = max(min(np.around(self.p1[self.st==1][n][0]), self.width-1), 0)
        y = max(min(np.around(self.p1[self.st==1][n][1]), self.height-1), 0)
        Z = max(min(1.0 / self.depth_mat[y][x], 10.0), 0.2)
        f = self.focal_length
        return np.array([[-f/Z, 0, x/Z, x*y/f, -(f+(x**2)/f), y],
                         [0, -f/Z, y/Z, f+(y**2)/f, -x*y/f, -x]])

    def img_cb_stereo(self, rgb, depth):
        """
        Receive image and depth topic, publish calculated twist topic.
        Using Lucas-Kanade pyramid method.
        """
        if self.first is True:
            self.old_gray = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='mono8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
            self.p0 = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
            self.mask = np.zeros_like(self.old_gray)
            self.first = False

        else:
            cv_gray = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='mono8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
            self.depth_mat = np.array(cv_depth).reshape(720, 1280)

            #print len(self.p0)
            if len(self.p0) < 100:
                self.p0 = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)

            self.p1, self.st, err = cv.calcOpticalFlowPyrLK(self.old_gray, cv_gray, self.p0, None, **self.lk_params)

            good_old = self.p0[self.st==1]
            good_new = self.p1[self.st==1]

            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                #self.mask = cv.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
                cv_gray = cv.circle(cv_gray, (a, b), 5, self.color[i].tolist(), -1)
            cv.imshow('optical flow', cv.add(cv_gray, self.mask))

            self.vx = (self.p1[self.st==1][:, 0] - self.p0[self.st==1][:, 0]) * self.rate
            self.vy = (self.p1[self.st==1][:, 1] - self.p0[self.st==1][:, 1]) * self.rate
            
            J = np.zeros((len(self.p1[self.st==1])*2, 6))
            x = np.zeros((len(self.p1[self.st==1])*2, 1))
            for i in range(0, len(self.p1[self.st==1])):
                J[i*2:(i+1)*2, :] = self.getStereoJ(i)
                x[i*2][0] = self.vx[i]
                x[i*2+1][0] = self.vy[i]
            v = np.dot(pseudoInverseMatrixL(J), x)

            self.twist.linear.x = v[0]
            self.twist.linear.y = v[1]
            self.twist.linear.z = v[2]
            self.twist.angular.x = v[3]
            self.twist.angular.y = v[4]
            self.twist.angular.z = v[5]
            self.twist_publisher.publish(self.twist)

            self.old_gray = cv_gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)

        cv.waitKey(1)


    def imu_cb(self, imu):
        """
        Get body angular velocity from imu(body frame) topic, then convert to camera frame.
        """
        self.p = -imu.angular.y
        self.q = -imu.angular.z
        self.r = imu.angular.x

    def rangefinder_cb(self, r):
        """
        Distance from the bottom of the drone to afloor.
        """
        self.distance = r

    def getMonoJ(self, n):
        x = max(min(np.around(self.p1[self.st==1][n][0]), self.width-1), 0)
        y = max(min(np.around(self.p1[self.st==1][n][1]), self.height-1), 0)
        Z = self.distance
        f = self.focal_length
        return np.array([[-f/Z, 0, x/Z, x*y/f, -(f+(x**2)/f), y],
                         [0, -f/Z, y/Z, f+(y**2)/f, -x*y/f, -x]])

    def img_cb_mono(self, rgb):
        """
        Receive image and depth topic, publish calculated twist topic.
        Using Lucas-Kanade pyramid method.
        """
        if self.first is True:
            self.old_gray = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='mono8')
            self.p0 = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)
            self.mask = np.zeros_like(self.old_gray)
            self.first = False

        else:
            # Change image topic into OpenCV image.
            cv_gray = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='mono8')

            # Search again if the number of features is few.
            if len(self.p0) < 15:
                self.p0 = cv.goodFeaturesToTrack(self.old_gray, mask = None, **self.feature_params)

            # Calculate optical flow using Lucas-Kanade method.
            self.p1, self.st, err = cv.calcOpticalFlowPyrLK(self.old_gray, cv_gray, self.p0, None, **self.lk_params)

            good_old = self.p0[self.st==1]
            good_new = self.p1[self.st==1]

            # # Show tracking features on the image.
            # for i, (new, old) in enumerate(zip(good_new, good_old)):
            #     a, b = new.ravel()
            #     c, d = old.ravel()
            #     #self.mask = cv.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
            #     cv_gray = cv.circle(cv_gray, (a, b), 5, self.color[i].tolist(), -1)
            # cv.imshow('optical flow', cv.add(cv_gray, self.mask))

            # Convert point difference to point velocity.
            self.vx = (self.p1[self.st==1][:, 0] - self.p0[self.st==1][:, 0]) * self.rate
            self.vy = (self.p1[self.st==1][:, 1] - self.p0[self.st==1][:, 1]) * self.rate

            # Calculate velocity in real world.
            J = np.zeros((len(self.p1[self.st==1])*2, 6))
            x = np.zeros((len(self.p1[self.st==1])*2, 1))
            for i in range(0, len(self.p1[self.st==1])):
                J[i*2:(i+1)*2, :] = self.getMonoJ(i)
                x[i*2][0] = self.vx[i]
                x[i*2+1][0] = self.vy[i]
            v = np.dot(pseudoInverseMatrixL(J), x)

            # Publish calculated velocity into ROS topic.
            self.twist.linear.x = v[0]
            self.twist.linear.y = v[1]
            self.twist.linear.z = v[2]
            self.twist.angular.x = v[3]
            self.twist.angular.y = v[4]
            self.twist.angular.z = v[5]
            self.twist_publisher.publish(self.twist)

            self.old_gray = cv_gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)

        cv.waitKey(1)

    
    def img_cb_fb(self, rgb, depth):
        """
        Optical flow using Farneback method
        """
        if self.first is True:
            self.old_gray = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='mono8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
            
            self.hsv = np.zeros((720, 1280, 3))
            self.hsv[..., 1] = 255
            self.first = False
        else:
            self.cv_gray = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='mono8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth, desired_encoding='32FC1')
            
            flow = cv.calcOpticalFlowFarneback(self.old_gray, self.cv_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

            mag, ang = cv.cartToPolar(flow[..., 0], flow[..., 1])
            self.hsv[..., 0] = ang / np.pi * 90
            self.hsv[..., 2] = cv.normalize(mag, None, 0, 255, cv.NORM_MINMAX)
            bgr = cv.cvtColor(self.hsv.astype(np.uint8), cv.COLOR_HSV2BGR)

            cv.imshow('optical flow', bgr)
            self.old_gray = self.cv_gray

            print 'ok'
            cv.waitKey(1)


    def __init__(self):
        rospy.init_node('riseq_estimation_opticalflow')

        self.bridge = cv_bridge.CvBridge()
        self.rate = 50
        self.r = rospy.Rate(self.rate)

        self.distance = 0.1

        self.first = True

        # Set the camera and optical flow method.
        camera = 'oCam'
        method = 'LucasKanade'

        self.twist_publisher = rospy.Publisher('/riseq/estimation/optical_flow', Twist, queue_size=10)
        self.twist = Twist()

        if camera == 'ZED':
            self.focal_length = 700
            self.width = 1280
            self.height = 720

            rgb_sub = message_filters.Subscriber('/zed/rgb/image_rect_color', Image)
            depth_sub = message_filters.Subscriber('/zed/depth/depth_registered', Image)
            img_sub = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 10)
            if method == 'LucasKanade':
                self.max_num = 200
                self.feature_params = dict(maxCorners = self.max_num, qualityLevel = 0.01, minDistance = 7, blockSize = 7)
                self.lk_params = dict(winSize = (15, 15), maxLevel = 2, criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
                self.color = np.random.randint(0, 255, (self.max_num, 3))
                img_sub.registerCallback(self.img_cb_lk)

        elif camera == 'oCam':
            self.focal_length = 968
            self.width = 1280
            self.height = 960

            if method == 'LucasKanade':
                # Set Haris detector parameters to track features.
                self.max_num = 200
                self.feature_params = dict(maxCorners = self.max_num, qualityLevel = 0.1, minDistance = 7, blockSize = 7)
                self.lk_params = dict(winSize = (15, 15), maxLevel = 2, criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
                self.color = np.random.randint(0, 255, (self.max_num, 3))

            rospy.Subscriber('/riseq/sensor/rangefinder', Image, self.rangefinder_cb)
            rospy.Subscriber('/camera/image_rect', Image, self.img_cb_mono)


    def loop(self):
        self.r.sleep()


if __name__ == "__main__":
    try:
        print "OpenCV: " + cv.__version__
        optical_flow = OpticalFlow()
        while not rospy.is_shutdown():
            optical_flow.loop()
    except rospy.ROSInterruptException:
        pass

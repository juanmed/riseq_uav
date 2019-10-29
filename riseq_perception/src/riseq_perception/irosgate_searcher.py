#!/usr/bin/env python

import rospy
import numpy as np
import irosgate_focallength_detector
from cv_bridge import CvBridge

import tf
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped


class IROSGateSearcher:
    def __init__(self):
        self.bridge = CvBridge()

        self.initial_yaw = 0
        self.yaw = 0
        self.theta = rospy.get_param("/perception/yaw_size", 30)

        self.dilate_iter = 2
        self.erode_iter = 4
        self.dilate_max = 5
        self.erode_min = 1

        self.irosgate_detector = irosgate_focallength_detector.IROSGateDetector(mode="eval", color_space='RGB')

        self.K = None
        self.D = None

        environment = rospy.get_param("environment", "real")
        if environment == "real":
            rospy.Subscriber("/zed/zed_node/left/camera_info", CameraInfo, self.init_param)
        elif environment == "simulator":
            rospy.Subscriber("/stereo/left/camera_info", CameraInfo, self.init_param)
        while self.K is None or self.D is None:
            rospy.sleep(0.1)
        self.irosgate_detector.K = self.K
        self.irosgate_detector.distCoeffs = self.D
        self.irosgate_detector.image_height = self.camera_height
        self.irosgate_detector.image_width = self.camera_width
        self.irosgate_detector.dilate_iter = self.dilate_iter
        self.irosgate_detector.erode_iter = self.erode_iter

        if environment == "real":
            rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.detect)
        elif environment == "simulator":
            rospy.Subscriber("/stereo/left/image_rect_color", Image, self.detect)

        self.wp_pub = rospy.Publisher("/riseq/perception/local_waypoint", PoseStamped, queue_size=10)

    def detect(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        R, t, R_exp, cnt, mask, cnts = self.irosgate_detector.detect(img, None)

        if t is not None:
            wp = PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = "local"
            wp.pose.position.x = t[0]
            wp.pose.position.y = t[1]
            wp.pose.position.z = t[2]

            R = np.concatenate((R, np.array([[0.0, 0.0, 0.0]])), axis=0)
            R = np.concatenate((R, np.array([[0.0, 0.0, 0.0, 1.0]]).T), axis=1)
            gate_quat = tf.transformations.quaternion_from_matrix(R)
            wp.pose.orientation.x = gate_quat[0]
            wp.pose.orientation.y = gate_quat[1]
            wp.pose.orientation.z = gate_quat[2]
            wp.pose.orientation.w = gate_quat[3]

            self.wp_pub.publish(wp)
            self.dilate_iter = 2
            self.erode_iter = 4

    def search_gate(self):
        print "search gate"
        if self.yaw < self.initial_yaw + 2 * np.pi:
            self.yaw = self.yaw + self.theta / 180.0 * np.pi
        else:
            self.yaw = self.initial_yaw
            if self.erode_iter > self.erode_min:
                self.erode_iter = self.erode_iter - 1
                self.irosgate_detector.erode_iter = self.erode_iter
            else:
                if self.dilate_iter < self.dilate_max:
                    self.dilate_iter = self.dilate_iter + 1
                    self.irosgate_detector.dilate_iter = self.dilate_iter
                else:
                    print "cannot find gate..."
                    self.dilate_iter = 2
                    self.erode_iter = 4
                    self.irosgate_detector.erode_iter = self.erode_iter
                    self.irosgate_detector.dilate_iter = self.dilate_iter
                    return False  # This means that there is no more step.
        return True

    def init_param(self, msg):
        self.camera_height = msg.height
        self.camera_width = msg.width

        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)


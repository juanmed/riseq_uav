#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from imutils import perspective

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseStamped

import lowpassfilter as lpf


class IROSGateDetector:
    def __init__(self):
        self.gate_width = 1.4  # m
        self.gate_height = 1.4  # m

        self.window_height = 1.4
        self.window_width = 1.4
        self.corners3D = np.zeros((4, 3))
        self.corners3D[0] = np.array([self.window_width / 2, 0.0, self.window_height / 2])
        self.corners3D[1] = np.array([-self.window_width / 2, 0.0, self.window_height / 2])
        self.corners3D[2] = np.array([-self.window_width / 2, 0.0, -self.window_height / 2])
        self.corners3D[3] = np.array([self.window_width / 2, 0.0, -self.window_height / 2])

        cutoff = rospy.get_param("/perception/cutoff", 0.001)
        self.lpf1_x = lpf.LowPassFilter(cutoff, 25)
        self.lpf1_y = lpf.LowPassFilter(cutoff, 25)
        self.lpf1_z = lpf.LowPassFilter(cutoff, 25)
        self.lpf2 = lpf.LowPassFilter(0.1, 25)

        # This params must be initialized with the best performing values
        self.canny_lt = 75
        self.canny_ht = 200
        self.canny_k = 5
        self.epsilon = 0.1  # should be between 0.02 to 0.15
        self.gauss_k = 5  # should be 3 or 5, but no more

        # For how to decide HSV color boundaries, look
        # https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
        green = [([60 - 20, 100, 40], [60 + 20, 255, 255])]
        blue = [([120 - 20, 100, 40], [120 + 20, 255, 255])]
        self.HSVboundaries = blue
        # self.HSVboundaries = [([160, 100, 40], [180, 255, 255]), #red
        #                      ([0, 100, 40], [30, 255, 255])]
        # ([25, 146, 190], [62, 174, 250]),
        # ([103, 86, 65], [145, 133, 128])]
        self.bridge = CvBridge()

        self.axis = np.float32([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]]).reshape(-1, 3)
        self.K = None
        self.D = None

        # rospy.Subscriber("/stereo/left/camera_info", CameraInfo, self.init_param)  # simulator
        rospy.Subscriber("/zed/zed_node/left/camera_info", CameraInfo, self.init_param)  # rect
        # rospy.Subscriber("/zed/zed_node/left_raw/camera_info", CameraInfo, self.init_param)  # raw
        while self.K is None:
            rospy.sleep(0.1)
        # rospy.Subscriber("/zed/zed_node/left_raw/image_raw_color", Image, self.detect)  # raw
        rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.detect)  # rect
        # rospy.Subscriber("/stereo/left/image_rect_color", Image, self.detect)  # simulator
        self.computed_wp_pub = rospy.Publisher("/riseq/perception/computed_position", PoseStamped, queue_size=10)
        self.computed_wp_lpf_pub = rospy.Publisher("/riseq/perception/computed_position_lpf", PoseStamped, queue_size=10)
        self.solvepnp_wp_pub = rospy.Publisher("/riseq/perception/solvepnp_position", PoseStamped, queue_size=10)
        self.img_with_det = rospy.Publisher("/riseq/perception/uav_image_with_detections", Image, queue_size=10)
        self.mask = rospy.Publisher("/riseq/perception/mask_image", Image, queue_size=10)

    def detect(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        # blur, filter colors, dilate and erode to define edges, and find contours
        blur = cv2.GaussianBlur(img, (self.gauss_k, self.gauss_k), 0)
        mask = self.filterColors(blur)

        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=1)

        inf, cnts, hrch = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        try:
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
        except:
            pass

        epsilon = 0.1
        squares = []
        corners2D = None
        R = None
        t = None
        cx = 0
        cy = 0

        for cnt in cnts:

            # approximate the contour
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon * peri, True)

            # if our approximated contour has four points, then we
            # can assume that we have found our square
            if len(approx) == 4:
                squares.append(approx)

        for square in squares:

            rect = cv2.minAreaRect(square)
            w, h = rect[1]
            # x,y,w,h = cv2.boundingRect(square)

            # set a lower threshold
            if ((w < 40) or (h < 40)):
                continue
            else:
                pass

            # verify width and height are similar
            aspect_ratio = w / h
            if (not ((aspect_ratio > 0.9) and (aspect_ratio < 1.1))):
                continue
            else:
                pass

            # verify area
            valid_area = self.verifyArea(w, h, square)
            if (not valid_area):
                continue
            else:
                pass

            box = cv2.boxPoints(rect)
            box = np.array(box, dtype="int")
            rect = perspective.order_points(box) # order points: top-left, top-right, down-right, down-left
                                                 # https://www.pyimagesearch.com/2016/03/21/ordering-coordinates-clockwise-with-python-and-opencv/
            square = square.reshape((-1, 2))  # square_couples[0][0].reshape((-1,2))
            square = perspective.order_points(square)

            #screenCnt = square.reshape((4, 2))

            # get centroid
            m = cv2.moments(square)
            try:
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])

                corners2D = square.astype('float32')
                # corners2D = np.concatenate((screenCnt2, centroid), axis = 0)
                R, t, R_exp = self.getPose(self.corners3D, corners2D)
                img = self.draw_frame(img, (cx, cy), R_exp, t)

            except ZeroDivisionError:
                pass

            break

        self.pub_center(img, cx, cy, corners2D, R, t)

        maskmsg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.mask.publish(maskmsg)

    def verifyArea(self, w, h, square):
        """
        Verify if boundingRect area defined by w,h is similar to the area inside
        the possible square profile found.
        Args:
            w: width of the bounding rect of the contour 'square'
            h: height of the bounding rect of the contour 'square'
            square: contour whose area is to be compared
        """
        valid_area = False
        area1 = w*h
        area2 = cv2.contourArea(cv2.convexHull(square))
        #print("w*h : {}  contourArea: {}, ratio: {:.2f}".format(area1, area2, area1/area2))
        if (((area1/area2) > 0.9) and ((area1/area2) < 1.1)):
            valid_area = True
        return valid_area

    def filterColors(self, img):
        """
        Get an RGB image, transfor to HSV, and filter only specific colors
        defined by HSV boundaries
        Args:
            img: image to transfor to from RGB to HSV
        Returns:
            mask: A 2-dimension boolean np.array whose elements are true
                  if the image pixel value in the same position of element is
                  inside the boundaries specified
        """

        # store here the final mask
        mask = np.zeros(img.shape[:2], dtype=bool)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        for (lower, upper) in self.HSVboundaries:
            # create NumPy arrays from the HSVboundaries
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            # find the colors within the specified HSVboundaries and apply
            # the mask
            mask = cv2.inRange(img, lower, upper) + mask

        return mask

    def getPose(self, points_3D, points_2D):
        """
        Estimate ellipse 3D pose based on 2D points and 3D points correspondence
        Args:
            points_3D (np.array): 3D points of the object
            points_2D (np.array): 2D points in the image plane
            scale (int):factor by which the original image's width and height was modified
        Returns:
            R (np.array): 3x3 rotation matrix following the opencv camera convention
            t (np.array): 3x1 translation vector
        """
        assert points_3D.shape[0] == points_2D.shape[0], 'points 3D and points 2D must have same number of vertices'
        points_2D = np.ascontiguousarray(points_2D[:, :2]).reshape((4, 1, 2))
        _, R_exp, t = cv2.solvePnP(points_3D, points_2D, self.K, distCoeffs=self.D, flags=cv2.SOLVEPNP_EPNP)
        R, _ = cv2.Rodrigues(R_exp)
        return R, t, R_exp

    def draw_frame(self, img, corner, R_exp, t):
        """
        Draw over the image a coordinate frame with origin in corner
        and axis extending to the points in imgpts.
        Args:
            img (np.array): image over which to draw the coordinate frame
            corner (tuple): tuple of coordinates of the center, (x,y)
            imgpts (np.array): coordinates of the other end of each of the three
                               axis

        Taken from: https://docs.opencv.org/master/d7/d53/tutorial_py_pose.html
        """
        imgpts, jac = cv2.projectPoints(self.axis, R_exp, t, self.K, self.D)

        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (255, 0, 0), 2)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 2)
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (0, 0, 255), 2)

        img = cv2.circle(img, corner, 5, (255, 0, 0), 3)
        return img

    def pub_center(self, img, cx, cy, corners2D, R, t):
        if corners2D is not None:
            px = corners2D[2][0] - corners2D[0][0]
            py = corners2D[2][1] - corners2D[0][1]

            dx = self.gate_width * (cx - self.camera_width / 2) / px
            dy = self.gate_height * (cy - self.camera_height / 2) / py

            f = self.K[0][0]  # f = fx = fy
            dz = self.gate_width / px * f

            # raw data
            x = np.abs(dz)
            y = -dx
            z = -dy

            wp = PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = ""
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.position.z = z

            # Low Pass Filter data
            x = self.lpf1_x.low_pass_filter(x)
            y = self.lpf1_y.low_pass_filter(y)
            z = self.lpf1_z.low_pass_filter(z)
            self.computed_wp_pub.publish(wp)

            wp = PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = ""
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.position.z = z
            self.computed_wp_lpf_pub.publish(wp)

        if t is not None:
            x = t[2][0]
            y = -t[0][0]
            z = -t[1][0]

            x = self.lpf2.low_pass_filter(x)

            wp = PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = ""
            wp.pose.position.x = x
            wp.pose.position.y = y
            wp.pose.position.z = z

            self.solvepnp_wp_pub.publish(wp)

        imgmsg = self.bridge.cv2_to_imgmsg(img, "rgb8")
        self.img_with_det.publish(imgmsg)

    def init_param(self, msg):
        self.camera_height = msg.height
        self.camera_width = msg.width

        self.K = np.array(msg.K).reshape(3, 3)
        self.D = np.array(msg.D)
        #self.D = np.array(msg.D)
        # self.R, self.P


if __name__ == "__main__":
    rospy.init_node('riseq_gatedetector', anonymous=True)
    ig = IROSGateDetector()

    #rospy.Subscriber("/stereo/left/camera_info", CameraInfo, ig.init_param)
    rospy.spin()

    if rospy.is_shutdown():
        cv2.destroyAllWindows()

    try:
        rospy.loginfo("UAV gate detector created")
    except rospy.ROSInterruptException:
        print("ROS Terminated.")

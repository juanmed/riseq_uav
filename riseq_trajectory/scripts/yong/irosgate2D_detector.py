#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64MultiArray


class IROSGateDetector:
    def __init__(self):
        self.window_width = 1.4  # m
        self.window_height = 1.4  # m

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
        self.HSVboundaries = green
        # self.HSVboundaries = [([160, 100, 40], [180, 255, 255]), #red
        #                      ([0, 100, 40], [30, 255, 255])]
        # ([25, 146, 190], [62, 174, 250]),
        # ([103, 86, 65], [145, 133, 128])]
        self.bridge = CvBridge()

        rospy.Subscriber("/stereo/left/image_color", Image, self.detect)
        self.wp_pub = rospy.Publisher("/riseq/perception/2D_position", PoseStamped, queue_size=10)
        self.img_with_det = rospy.Publisher("/riseq/perception/uav_image_with_detections2d", Image, queue_size=10)
        self.size_pub = rospy.Publisher("/riseq/perception/gate_pixel_size", Int64MultiArray, queue_size=10)

    def detect(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

        # blur, filter colors, dilate and erode to define edges, and find contours
        blur = cv2.GaussianBlur(img, (self.gauss_k, self.gauss_k), 0)
        mask = self.filterColors(blur)

        #mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.erode(mask, None, iterations=1)

        inf, cnts, hrch = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        try:
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
        except:
            pass

        epsilon = 0.1
        squares = []
        corners2D = None
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

            screenCnt = square.reshape((4, 2))

            # get centroid
            m = cv2.moments(screenCnt)
            try:
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])

                centroid = np.array([[cx, cy]])
                # corners2D = np.concatenate((screenCnt2, centroid), axis = 0)

                corners2D = np.concatenate((centroid, screenCnt), axis=0)
                cv2.circle(img, (cx, cy), 10, (255, 0, 0), 1)


            except ZeroDivisionError:
                pass

            break

        self.pub_center(img, cx, cy, corners2D)

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

    def pub_center(self, img, cx, cy, corners2D):
        if corners2D is not None:
            wp = PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = ""

            wp.pose.position.x = cx
            wp.pose.position.y = cy
            self.wp_pub.publish(wp)

            size = Int64MultiArray()

            px = corners2D[3][0] - corners2D[1][0]
            py = corners2D[3][1] - corners2D[1][1]

            size.data.append(int(px))
            size.data.append(int(py))
            self.size_pub.publish(size)

        imgmsg = self.bridge.cv2_to_imgmsg(img, "rgb8")
        self.img_with_det.publish(imgmsg)

    def init_param(self, msg):
        self.camera_height = msg.height
        self.camera_width = msg.width

        self.K = msg.K
        self.D = msg.D
        # self.R, self.P


if __name__ == "__main__":
    rospy.init_node('riseq_2d', anonymous=True)

    ig = IROSGateDetector()

    #rospy.Subscriber("/stereo/left/camera_info", CameraInfo, ig.init_param)
    rospy.spin()

    if rospy.is_shutdown():
        cv2.destroyAllWindows()

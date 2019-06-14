#!/usr/bin/env python

"""
author:  author's name
version: version of this script
brief: a description of the functionality this script implements

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

from detector import *
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image 
from nav_msgs.msg import Path


class DRLGatePoseDetector():

    def __init__(self):
        """
        Initialize CNN for gate pose detection
        """

        self.gate_pose_pub = rospy.Publisher("riseq/perception/uav_waypoint", Path, queue_size = 10)
        
        self.gate_bbox_img_pub = rospy.Publisher("riseq/perception/gate_bbox", Image, queue_size = 10)

        self.frontCamera_Mono = rospy.Subscriber("/pelican/camera_nadir/image_raw", Image, self.predict_gate_pose)

        self.gateDetector = GateDetector()

        self.bridge = CvBridge()

    def predict_gate_pose(self, image_msg):
        """
        Receive an image, pass it through the network, and get the 
        estimated gate translation and rotation in camera frame.

        Args:
            image_msg: RGB image
        Returns:
            The detected translation as a 3-vector (np.array, [x,y,z]) and 
            orientation as a quaternion 4-vector(np.array, [x,y,z,w])
        """

        

        print("Type: {}, Len: {}, Width: {}, Height: {}, Enc: {}".format(type(image_msg.data), len(image_msg.data), image_msg.width, image_msg.height, image_msg.encoding))
        #img = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        img = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
        img_copy = img.copy()
        R, t, bbox_img, conf = self.gateDetector.predict(img)
        print("R: {},\n t: {}, conf: {:.3f}".format(R, t/3.45, conf))



        #waypoints = Path()
        #waypoints.header.stamp = rospy.Time.now()
        #waypoints.header.frame_id = ""
        #self.gate_pose_pub.publish(waypoints)

        #img = Image()
        #img.header.stamp = rospy.Time.now()
        #img.header.frame_id = ""
        #img.height = bbox_img.shape[0]
        #img.width = bbox_img.shape[1]
        #img.encoding = image_msg.encoding
        #img.data = CvBridge().cv2_to_imgmsg(bbox_img, "rgb8")
        img = self.bridge.cv2_to_imgmsg(bbox_img, "rgb8")
        self.gate_bbox_img_pub.publish(img)

        #rospy.loginfo(waypoints)






if __name__ == '__main__':
    try:
        rospy.init_node("riseq_gate_pose_publisher", anonymous = True)

        gate_pose_publisher = DRLGatePoseDetector()

        rospy.loginfo('Gate Pose Publisher Started')
        rospy.spin()
        rospy.loginfo('Gate Pose Publisher Terminated')     

    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass

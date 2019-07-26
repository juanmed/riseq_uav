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
import tf
import cv2

#from detector import *
from window_detector import WindowDetector
from ellipse_detector import EllipseDetector
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image 
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class MonoWaypointDetector():

    def __init__(self):
        """
        Initialize CNN for gate pose detection
        """

        self.waypoint_pub = rospy.Publisher("riseq/perception/uav_mono_waypoint", Path, queue_size = 10)
        
        self.img_dect_pub = rospy.Publisher("riseq/perception/uav_image_with_detections", Image, queue_size = 10)

        #self.frontCamera_Mono = rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.estimate_object_pose)
        self.frontCamera_Mono = rospy.Subscriber("/pelican/camera_nadir/image_raw", Image, self.estimate_object_pose)
        
        self.bridge = CvBridge()

        #self.init_pose = rospy.get_param("riseq/init_pose")
        #self.init_pose = [float(a) for a in self.init_pose]
        self.mode = rospy.get_param("riseq/monocular_cv", 'disable')
        self.max_size = 460


        # AI Challenge detectors
        self.wd = WindowDetector(mode="eval")
        self.pd = EllipseDetector(mode = "eval")

        # ADR Gate Detector
        #cfg_file = rospy.get_param("riseq/gate_pose_nn_cfg")
        #ply_file = rospy.get_param("riseq/gate_pose_nn_ply")
        #wgt_file = rospy.get_param("riseq/gate_pose_nn_wgt")
        #data_file = rospy.get_param("riseq/gate_pose_nn_data")
        #self.gateDetector = GateDetector(cfg_file, ply_file, wgt_file, data_file)

    def estimate_object_pose(self, image_msg):
        """
        Receive an image, pass it through the network, and get the 
        estimated gate translation and rotation in camera frame.

        Args:
            image_msg: RGB image
        Returns:
            The detected translation as a 3-vector (np.array, [x,y,z]) and 
            orientation as a quaternion 4-vector(np.array, [x,y,z,w])
        """

        # update MonoWaypointDetector
        self.mode = rospy.get_param("riseq/monocular_cv", 'disable')

        if(self.mode != 'disable'):

            img = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
            
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = ""

            wp = PoseStamped()
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = ""
            
            if(self.mode == 'window'):

                R, t, R_exp, cnt = self.wd.detect(img.copy(), self.max_size)

                if cnt is not None:

                    img = cv2.drawContours(img, [cnt[1:]], -1, (255,0,0), 3)
                    img = self.wd.draw_frame(img, (cnt[0][0],cnt[0][1]), R_exp, t)
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0]])), axis = 0)
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0, 1.0]]).T ), axis = 1)
                    gate_quat = tf.transformations.quaternion_from_matrix(R)

                    # gate waypoint
                    wp.pose.position.x = t[2][0]
                    wp.pose.position.y = -t[0][0]
                    wp.pose.position.z = t[1][0]
                    wp.pose.orientation.x = gate_quat[0]
                    wp.pose.orientation.y = gate_quat[1]
                    wp.pose.orientation.z = gate_quat[2]
                    wp.pose.orientation.w = gate_quat[3]

                    path.poses = [wp]

            elif(self.mode == 'pipe'):

                ellipses = self.pd.detect(img.copy(), self.max_size)
                
                if len(ellipses) == 1:
                    R, t, R_exp, e = ellipses[0]
                    img = cv2.ellipse(img, e, (255,0,0), 2)
                    img = self.pd.draw_frame(img, (e[0][0],e[0][1]), R_exp, t)

                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0]])), axis = 0)
                    R = np.concatenate((R, np.array([[0.0, 0.0, 0.0, 1.0]]).T ), axis = 1)
                    gate_quat = tf.transformations.quaternion_from_matrix(R)

                    # gate waypoint
                    wp.pose.position.x = t[2][0]
                    wp.pose.position.y = -t[0][0]
                    wp.pose.position.z = t[1][0]
                    wp.pose.orientation.x = gate_quat[0]
                    wp.pose.orientation.y = gate_quat[1]
                    wp.pose.orientation.z = gate_quat[2]
                    wp.pose.orientation.w = gate_quat[3]

                    path.poses = [wp]

            elif(self.mode == 'gate'):
                R, t, img, conf = self.gateDetector.predict(img.copy())

                gate_x = rospy.get_param("riseq/gate_x", 0.0)
                gate_y = rospy.get_param("riseq/gate_y", 0.0)
                gate_z = rospy.get_param("riseq/gate_z", 0.0)
                gate_roll = rospy.get_param("riseq/gate_roll", 0.0)
                gate_pitch = rospy.get_param("riseq/gate_pitch", 0.0)
                gate_yaw = rospy.get_param("riseq/gate_yaw", 0.0)

                gate_quat = tf.transformations.quaternion_from_euler(0.0, gate_pitch, gate_roll, axes='rzyx')
            
                # gate waypoint
                wp.pose.position.x = gate_x
                wp.pose.position.y = gate_y
                wp.pose.position.z = gate_z
                wp.pose.orientation.x = gate_quat[0]
                wp.pose.orientation.y = gate_quat[1]
                wp.pose.orientation.z = gate_quat[2]
                wp.pose.orientation.w = gate_quat[3]

            else:
                rospy.loginfo("Monocular Object Detector mode non-existent.")
                
            self.waypoint_pub.publish(path)
            img = self.bridge.cv2_to_imgmsg(img, "rgb8")
            self.img_dect_pub.publish(img)

        else:
            # Do nothing
            pass

        #print("Type: {}, Len: {}, Width: {}, Height: {}, Enc: {}".format(type(image_msg.data), len(image_msg.data), image_msg.width, image_msg.height, image_msg.encoding))
        #img = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)



        #img = Image()
        #img.header.stamp = rospy.Time.now()
        #img.header.frame_id = ""
        #img.height = bbox_img.shape[0]
        #img.width = bbox_img.shape[1]
        #img.encoding = image_msg.encoding
        #img.data = CvBridge().cv2_to_imgmsg(bbox_img, "rgb8")


        #rospy.loginfo(waypoints)

def gate_pose_publisher():
    try:
        rospy.init_node("riseq_gate_pose_publisher", anonymous = True)


        
        monocular_waypoint_publisher = MonoWaypointDetector()

        rospy.loginfo('Gate Pose Publisher Started')
        rospy.spin()
        rospy.loginfo('Gate Pose Publisher Terminated')     

    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass

if __name__ == '__main__':
    gate_pose_publisher()


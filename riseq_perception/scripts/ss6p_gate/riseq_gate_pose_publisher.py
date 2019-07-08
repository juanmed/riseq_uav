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

from detector import *
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image 
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class DRLGatePoseDetector():

    def __init__(self):
        """
        Initialize CNN for gate pose detection
        """

        self.gate_pose_pub = rospy.Publisher("riseq/planning/uav_waypoint", Path, queue_size = 10)
        
        self.gate_bbox_img_pub = rospy.Publisher("riseq/perception/gate_bbox", Image, queue_size = 10)

        self.frontCamera_Mono = rospy.Subscriber("/iris/camera_nadir/image_raw", Image, self.predict_gate_pose)

        cfg_file = rospy.get_param("riseq/gate_pose_nn_cfg")
        ply_file = rospy.get_param("riseq/gate_pose_nn_ply")
        wgt_file = rospy.get_param("riseq/gate_pose_nn_wgt")
        data_file = rospy.get_param("riseq/gate_pose_nn_data")

        self.gateDetector = GateDetector(cfg_file, ply_file, wgt_file, data_file)

        self.bridge = CvBridge()

        self.init_pose = rospy.get_param("riseq/init_pose")
        self.init_pose = [float(a) for a in self.init_pose]

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

        

        #print("Type: {}, Len: {}, Width: {}, Height: {}, Enc: {}".format(type(image_msg.data), len(image_msg.data), image_msg.width, image_msg.height, image_msg.encoding))
        #img = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        img = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
        img_copy = img.copy()
        R, t, bbox_img, conf = self.gateDetector.predict(img)
        #print("R: {},\n t: {}, conf: {:.3f}".format(R, t/3.45, conf))


        gate_x = rospy.get_param("riseq/gate_x", 0.0)
        gate_y = rospy.get_param("riseq/gate_y", 0.0)
        gate_z = rospy.get_param("riseq/gate_z", 0.0)
        gate_roll = rospy.get_param("riseq/gate_roll", 0.0)
        gate_pitch = rospy.get_param("riseq/gate_pitch", 0.0)
        gate_yaw = rospy.get_param("riseq/gate_yaw", 0.0)

        gate_quat = tf.transformations.quaternion_from_euler(0.0, gate_pitch, gate_roll, axes='rzyx')



        wps = []

        # drone init pose waypoint
        wp1 = PoseStamped()
        wp1.pose.position.x = self.init_pose[0]
        wp1.pose.position.y = self.init_pose[1]
        wp1.pose.position.z = self.init_pose[2]
        wp1.pose.orientation.x = self.init_pose[3]
        wp1.pose.orientation.y = self.init_pose[4]
        wp1.pose.orientation.z = self.init_pose[5]
        wp1.pose.orientation.w = self.init_pose[6]
        wps.append(wp1)

        wp3 = PoseStamped()
        wp3.pose.position.x = 0.0
        wp3.pose.position.y = 0.0
        wp3.pose.position.z = gate_z
        wp3.pose.orientation.x = gate_quat[0]
        wp3.pose.orientation.y = gate_quat[1]
        wp3.pose.orientation.z = gate_quat[2]
        wp3.pose.orientation.w = gate_quat[3]       
        #wps.append(wp3)

        # gate waypoint
        wp2 = PoseStamped()
        wp2.pose.position.x = gate_x
        wp2.pose.position.y = gate_y
        wp2.pose.position.z = gate_z
        wp2.pose.orientation.x = gate_quat[0]
        wp2.pose.orientation.y = gate_quat[1]
        wp2.pose.orientation.z = gate_quat[2]
        wp2.pose.orientation.w = gate_quat[3]
        wps.append(wp2)

        # create Path
        waypoints = Path()
        waypoints.header.stamp = rospy.Time.now()
        waypoints.header.frame_id = ""
        waypoints.poses = wps
        self.gate_pose_pub.publish(waypoints)

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

def gate_pose_publisher():
    try:
        rospy.init_node("riseq_gate_pose_publisher", anonymous = True)


        
        gate_pose_publisher = DRLGatePoseDetector()

        rospy.loginfo('Gate Pose Publisher Started')
        rospy.spin()
        rospy.loginfo('Gate Pose Publisher Terminated')     

    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass




if __name__ == '__main__':
    gate_pose_publisher()


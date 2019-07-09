#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: This node publishes /tf topic to connect frames of Visual Odometry and frames of ZED camera.

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
import tf
from tf2_msgs.msg import TFMessage


if __name__ == "__main__":
    try:
        rospy.init_node('riseq_estimation_tf_publisher')
        br = tf.TransformBroadcaster()
        r = rospy.Rate(1000)

        # Visual odometry method to use. SVO, ORB_SLAM2
        method = "ZED mini"
        realsense = True

        while not rospy.is_shutdown():
            if method == "SVO":
                br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "cam_pos", "base_link")
            elif method == "ORB_SLAM2":
                br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "base_link", "camera_link")
            elif method == "ZED mini":
                br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "map", "world")
                br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "local_origin", "world")

            if realsense == True:
                br.sendTransform((0.1, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "camera_link", "fcu")

            r.sleep()

    except rospy.ROSInterruptException:
        pass

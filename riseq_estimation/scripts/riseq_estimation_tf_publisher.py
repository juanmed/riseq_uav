#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: This node publishes /tf topic to connect world frame of SVO and map frame of ZED camera
       not to ZED odometry data on octomap.

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

        while not rospy.is_shutdown():
            br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "cam_pos", "zed_left_camera_optical_frame")
            r.sleep()

    except rospy.ROSInterruptException:
        pass
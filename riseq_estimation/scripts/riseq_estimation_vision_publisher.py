#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: This code is publishing pose from visual odometry to PX4. Changes the pose coordinate from SLAM to NED frame.

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
from geometry_msgs.msg import PoseStamped

class VisionPublisher():
    def pose_cb(self, vo_pose):
        """
        Convert the coordinate of the visual odometry pose into mavros message.
        MAVROS uses ENU and flu frames.
        """
        if self.method == "ORB_SLAM2":
            self.px4_pose.pose.position.x = vo_pose.pose.position.x
            self.px4_pose.pose.position.y = vo_pose.pose.position.y
            self.px4_pose.pose.position.z = vo_pose.pose.position.z
            self.px4_pose.pose.orientation.x = -vo_pose.pose.orientation.x
            self.px4_pose.pose.orientation.y = -vo_pose.pose.orientation.y
            self.px4_pose.pose.orientation.z = -vo_pose.pose.orientation.z
            self.px4_pose.pose.orientation.w = -vo_pose.pose.orientation.w
            self.px4_pose.header.stamp = rospy.Time.now()

        self.pose_publisher.publish(self.px4_pose)


    def __init__(self):
        rospy.init_node('riseq_vision_px4_publisher')
        self.rate = 100
        self.r = rospy.Rate(self.rate)

        self.px4_pose = PoseStamped()
        self.px4_pose.header.frame_id = "map"
        self.pose_publisher = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/riseq/estimation/vo_pose', PoseStamped, self.pose_cb)

        self.method = "ORB_SLAM2"
        

    def loop(self):
        self.r.sleep()

if __name__ == "__main__":
    try:
        vision_publisher = VisionPublisher()
        while not rospy.is_shutdown():
            vision_publisher.loop()

    except rospy.ROSInterruptException:
        pass

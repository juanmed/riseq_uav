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
    def pose_cb(self, pose):
        """
        Convert the coordinate of the visual odometry pose into mavros message.
        MAVLINK uses NED frames.
        """
        self.pose_px4.pose.position.x = pose.pose.position.x
        self.pose_px4.pose.position.y = pose.pose.position.y
        self.pose_px4.pose.position.z = pose.pose.position.z
        self.pose_px4.pose.orientation.x = -pose.pose.orientation.x
        self.pose_px4.pose.orientation.y = -pose.pose.orientation.y
        self.pose_px4.pose.orientation.z = -pose.pose.orientation.z
        self.pose_px4.pose.orientation.w = -pose.pose.orientation.w
        self.pose_px4.header.stamp = rospy.Time.now()

        self.pose_publisher.publish(self.pose_px4)


    def __init__(self):
        rospy.init_node('riseq_vision_px4_publisher')
        self.rate = 100
        self.r = rospy.Rate(self.rate)

        self.pose_px4 = PoseStamped()
        self.pose_px4.header.frame_id = "map"
        self.pose_publisher = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/orb_slam2/pose', PoseStamped, self.pose_cb)


    def loop(self):
        self.r.sleep()

if __name__ == "__main__":
    try:
        vision_publisher = VisionPublisher()
        while not rospy.is_shutdown():
            vision_publisher.loop()

    except rospy.ROSInterruptException:
        pass

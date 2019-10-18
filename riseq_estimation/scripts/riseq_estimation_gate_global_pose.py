#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: 

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

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from eugene_kinematics import q2r


class GateGlobal:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('riseq_estimation_gate_global_pose')
        self.frequency = 1.0
        self.r = rospy.Rate(self.frequency)

        self.camera_on_body = np.array([0.185, 0.0, 0.0])
        self.cur_pose = PoseStamped()
        self.cur_pose.pose.orientation.w = 1.0

        # Publisher, Subscriber
        self.cur_pose_pub = rospy.Publisher('/riseq/drone/cur_pose', PoseStamped, queue_size=10)
        self.gate_global_pose_pub = rospy.Publisher('/riseq/gate/raw/global_pose', PoseStamped, queue_size=10)
        self.gate_global_pose_pub2 = rospy.Publisher('/riseq/gate/lpf_body/global_pose', PoseStamped, queue_size=10)
        self.gate_global_pose_pub3 = rospy.Publisher('/riseq/gate/lpf_global/global_pose', PoseStamped, queue_size=10)

        #rospy.Subscriber('/zed/zed_node/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        #rospy.Subscriber('/riseq/perception/solvepnp_position', PoseStamped, self.gate_cb)
        rospy.Subscriber('/riseq/perception/computed_position', PoseStamped, self.gate_cb)

        # LPF parameters
        self.last_time = rospy.Time.now()

        self.gate_body = PoseStamped()
        self.gate_body.pose.orientation.w = 1.0
        self.gate_global2 = PoseStamped()
        self.gate_global2.pose.orientation.w = 1.0
        
        self.gate_global3 = PoseStamped()
        self.gate_global3.pose.orientation.w = 1.0
        
        sampling_frequency = 60.0
        cutoff_frequency = 5.0
        self.alpha = (1.0/sampling_frequency)/((1.0/sampling_frequency)+(1.0/cutoff_frequency))

    def loop(self):
        self.r.sleep()

    def pose_cb(self, msg):
        self.cur_pose.header.stamp = rospy.Time.now()
        self.cur_pose.pose.position.x = msg.pose.position.x
        self.cur_pose.pose.position.y = msg.pose.position.y
        self.cur_pose.pose.position.z = msg.pose.position.z
        self.cur_pose.pose.orientation.x = msg.pose.orientation.x
        self.cur_pose.pose.orientation.y = msg.pose.orientation.y
        self.cur_pose.pose.orientation.z = msg.pose.orientation.z
        self.cur_pose.pose.orientation.w = msg.pose.orientation.w
        self.cur_pose_pub.publish(self.cur_pose)

    def gate_cb(self, msg):
        # Raw
        R = q2r(self.cur_pose.pose.orientation.w, self.cur_pose.pose.orientation.x, self.cur_pose.pose.orientation.y, self.cur_pose.pose.orientation.z)
        p = np.dot(R, np.array([[msg.pose.position.x + self.camera_on_body[0]], [msg.pose.position.y + self.camera_on_body[1]], [msg.pose.position.z + self.camera_on_body[2]]]))
        gate_global_pose = PoseStamped()
        gate_global_pose.header.stamp = rospy.Time.now()
        gate_global_pose.pose.position.x = p[0][0] + self.cur_pose.pose.position.x
        gate_global_pose.pose.position.y = p[1][0] + self.cur_pose.pose.position.y
        gate_global_pose.pose.position.z = p[2][0] + self.cur_pose.pose.position.z
        gate_global_pose.pose.orientation.w = 1.0

        # Body LPF
        R = q2r(self.cur_pose.pose.orientation.w, self.cur_pose.pose.orientation.x, self.cur_pose.pose.orientation.y, self.cur_pose.pose.orientation.z)
        self.gate_body.pose.position.x = (1-self.alpha)*self.gate_body.pose.position.x + self.alpha*msg.pose.position.x
        self.gate_body.pose.position.y = (1-self.alpha)*self.gate_body.pose.position.y + self.alpha*msg.pose.position.y
        self.gate_body.pose.position.z = (1-self.alpha)*self.gate_body.pose.position.z + self.alpha*msg.pose.position.z
        p = np.dot(R, np.array([[self.gate_body.pose.position.x + self.camera_on_body[0]], [self.gate_body.pose.position.y + self.camera_on_body[1]], [self.gate_body.pose.position.z + self.camera_on_body[2]]]))
        self.gate_global2.pose.position.x = p[0][0] + self.cur_pose.pose.position.x
        self.gate_global2.pose.position.y = p[1][0] + self.cur_pose.pose.position.y
        self.gate_global2.pose.position.z = p[2][0] + self.cur_pose.pose.position.z
        self.gate_global2.pose.orientation.w = 1.0

        # Global LPF
        # if (rospy.Time.now().to_sec() - self.last_time.to_sec()) > 1.0:
        #     self.gate_global3.pose.position.x = gate_global_pose.pose.position.x
        #     self.gate_global3.pose.position.y = gate_global_pose.pose.position.y
        #     self.gate_global3.pose.position.z = gate_global_pose.pose.position.z
        # else:
        #     self.gate_global3.pose.position.x = (1-self.alpha)*self.gate_global3.pose.position.x + self.alpha*gate_global_pose.pose.position.x
        #     self.gate_global3.pose.position.y = (1-self.alpha)*self.gate_global3.pose.position.y + self.alpha*gate_global_pose.pose.position.y
        #     self.gate_global3.pose.position.z = (1-self.alpha)*self.gate_global3.pose.position.z + self.alpha*gate_global_pose.pose.position.z
        self.gate_global3.pose.position.x = (1-self.alpha)*self.gate_global3.pose.position.x + self.alpha*gate_global_pose.pose.position.x
        self.gate_global3.pose.position.y = (1-self.alpha)*self.gate_global3.pose.position.y + self.alpha*gate_global_pose.pose.position.y
        self.gate_global3.pose.position.z = (1-self.alpha)*self.gate_global3.pose.position.z + self.alpha*gate_global_pose.pose.position.z

        gate_global_pose.header.stamp = rospy.Time.now()
        self.gate_global_pose_pub.publish(gate_global_pose)

        self.gate_global2.header.stamp = rospy.Time.now()
        self.gate_global_pose_pub2.publish(self.gate_global2)

        self.gate_global3.header.stamp = rospy.Time.now()
        self.gate_global_pose_pub3.publish(self.gate_global3)

        self.last_time = rospy.Time.now()


if __name__ == "__main__":
    try:
        gate_global = GateGlobal()
        while not rospy.is_shutdown():
            gate_global.loop()

    except rospy.ROSInterruptException:
        pass

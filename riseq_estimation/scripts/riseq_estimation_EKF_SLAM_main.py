#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: EKF-SLAM using VIO and Gate Detection.

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
from scipy.spatial.transform import Rotation
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class EKFSLAM:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('riseq_estimation_EKF_SLAM')
        self.frequency = 10.0
        self.r = rospy.Rate(self.frequency)

        ## Gate parameters & Check
        self.gate_v = 0
        self.gate_h_l = 1
        self.gate_h_r = 2
        self.gate_pose = np.array([rospy.get_param('/gates/gate_vertical/up', [6, 0.0, 1.9]),
                                   rospy.get_param('/gates/gate_horizontal/left', [-4, 2.0, 1.7]),
                                   rospy.get_param('/gates/gate_horizontal/right', [-4, 3.4, 1.7])])
        self.gate_pose[0][2] = 1.9
        self.gate_observing = -1
        self.gate_last = -1
        self.gate_first = rospy.get_param('/gates/at_first', self.gate_v)
        if self.gate_first != self.gate_v:
            for i in range(0, 3):
                for j in range(0, 2):
                    self.gate_pose[i][j] = -self.gate_pose[i][j]
        ##

        ## State
        self.F = np.eye(4)
        self.x_pre = np.zeros((4, 1))
        self.x_est = np.array([[0.0],                               # drone gate_global_pose position x
                               [0.0],                               #                                 y
                               [0.0],                               # drone VO drift x
                               [0.0]])                              #                y
        self.P_pre = np.eye(4) * 1e-2
        self.P_est = np.array([[1e-2, 0.0, 0.0, 0.0],
                               [0.0, 1e-2, 0.0, 0.0],
                               [0.0, 0.0, 1e-2, 0.0],
                               [0.0, 0.0, 0.0, 1e-2]])

        self.B = np.zeros((4, 2))
        self.B[0][0] = 1.0
        self.B[1][1] = 1.0
        self.u = np.zeros((2, 1))                                   # odometry(traveled distance) as input. B*u=I*(v*dt)
        self.Q = np.eye(4) * 1e-5

        self.z = np.array([[0.0],                                   # VO position x
                           [0.0],                                   #             y
                           [0.0],                                   # distance to the gate in local frame x(depth)
                           [0.0]])                                  #                                     y(horizon)
        self.H_full = np.array([[1.0, 0.0, 1.0, 0.0],               # VO_poseition = real_position + drift
                                [0.0, 1.0, 0.0, 1.0],
                                [-1.0, 0.0, 0.0, 0.0],              # gate_camera_frame = gate_global_frame - drone_global_frame
                                [0.0, -1.0, 0.0, 0.0]])
        self.R = np.eye(4) * 1e-4
        self.R[2:4, 2:4] = np.eye(2) * 1e-4
        ##

        ## Publisher, Subscriber
        self.comp_pose_pub = rospy.Publisher('/riseq/drone/pose', PoseStamped, queue_size=10)
        self.drift_pub = rospy.Publisher('/riseq/drone/vo_drift', PoseStamped, queue_size=10)
        self.gate_pose_pub = rospy.Publisher('/riseq/gate/pose', PoseStamped, queue_size=10)
        self.gate_seeing_pub = rospy.Publisher('/riseq/gate/observing', String, queue_size=10)

        self.local_pose = PoseStamped()
        self.local_pose.header.frame_id = 'map'
        self.local_pose.pose.orientation.w = 1.0

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.vo_pose_cb)
        # rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.vo_pose_cb)
        # rospy.Subscriber('/zed/zed_node/pose', PoseStamped, self.vo_pose_cb)
        rospy.Subscriber('/riseq/gate/lpf_global/camera_pose', PoseStamped, self.gate_cb)
        ##

    def loop(self):
        ## Update VO measurements
        self.z[0][0] = self.local_pose.pose.position.x
        self.z[1][0] = self.local_pose.pose.position.y
        ##

        ## Kalman Filter
        if self.gate_observing < 0:
            # self.x_pre = np.dot(self.F, self.x_est) + np.dot(self.B, self.u)
            # self.P_pre = np.linalg.multi_dot([self.F, self.P_est, self.F.T]) + self.Q
            # self.P_pre[2:4, 2:4] = np.eye(2) * 1e-4
            # H = self.H_full[0:2, :]
            # K = np.linalg.multi_dot([self.P_pre, H.T, np.linalg.inv(np.linalg.multi_dot([H, self.P_pre, H.T]) + self.R[0:2, 0:2])])
            # self.x_est = self.x_pre + np.dot(K, self.z[0:2, :] - np.dot(H, self.x_pre))
            # self.P_est = np.dot(np.eye(4) - np.dot(K, H), self.P_pre)
            self.x_est[0:2, :] = self.z[0:2, :] - self.x_est[2:4, :]
        else:
            self.x_pre = np.dot(self.F, self.x_est) + np.dot(self.B, self.u)
            self.P_pre = np.linalg.multi_dot([self.F, self.P_est, self.F.T]) + self.Q
            # self.P_pre[2:4, 2:4] = np.eye(2) * 1e1
            H = self.H_full
            K = np.linalg.multi_dot([self.P_pre, H.T, np.linalg.pinv(np.linalg.multi_dot([H, self.P_pre, H.T]) + self.R)])
            self.x_est = self.x_pre + np.dot(K, self.z - (np.array([[0.0],
                                                                    [0.0],
                                                                    [self.gate_pose[self.gate_observing][0]],
                                                                    [self.gate_pose[self.gate_observing][1]]]) + np.dot(H, self.x_pre)))
            self.P_est = np.dot(np.eye(4) - np.dot(K, H), self.P_pre)

        self.gate_observing = -1
        ##

        ## Publish data
        drift = PoseStamped()
        drift.header.stamp = rospy.Time.now()
        drift.header.frame_id = 'map'
        drift.pose.position.x = self.x_est[2][0]
        drift.pose.position.y = self.x_est[3][0]
        self.drift_pub.publish(drift)

        gate = PoseStamped()
        gate.header.stamp = rospy.Time.now()
        gate.header.frame_id = 'map'
        gate.pose.orientation.w = 1.0
        if self.gate_observing > -1:
            gate.pose.position.x = self.gate_pose[self.gate_observing][0]
            gate.pose.position.y = self.gate_pose[self.gate_observing][1]
            gate.pose.position.z = self.gate_pose[self.gate_observing][2]
            self.gate_pose_pub.publish(gate)
        ##

        self.r.sleep()

    def vo_pose_cb(self, msg):
        self.local_pose.header.stamp = msg.header.stamp
        self.local_pose.pose.position.x = msg.pose.position.x
        self.local_pose.pose.position.y = msg.pose.position.y
        self.local_pose.pose.position.z = msg.pose.position.z
        self.local_pose.pose.orientation.x = msg.pose.orientation.x
        self.local_pose.pose.orientation.y = msg.pose.orientation.y
        self.local_pose.pose.orientation.z = msg.pose.orientation.z
        self.local_pose.pose.orientation.w = msg.pose.orientation.w

        # Publish compensated pose
        comp_pose = PoseStamped()
        comp_pose.header.stamp = msg.header.stamp
        comp_pose.header.frame_id = 'map'
        comp_pose.pose.position.x = msg.pose.position.x - self.x_est[2][0]
        comp_pose.pose.position.y = msg.pose.position.y - self.x_est[3][0]
        comp_pose.pose.position.z = msg.pose.position.z
        comp_pose.pose.orientation.x = msg.pose.orientation.x
        comp_pose.pose.orientation.y = msg.pose.orientation.y
        comp_pose.pose.orientation.z = msg.pose.orientation.z
        comp_pose.pose.orientation.w = msg.pose.orientation.w
        self.comp_pose_pub.publish(comp_pose)

    def gate_cb(self, msg):
        gate_seeing = String()
        gate_global_pose = np.array([[self.x_est[0][0] + msg.pose.position.x], [self.x_est[1][0] + msg.pose.position.y], [self.local_pose.pose.position.z + msg.pose.position.z]])

        ## Calculate distance to the gates
        dist_v = np.linalg.norm(gate_global_pose[0:2][:] - np.array([[self.gate_pose[self.gate_v][0]], [self.gate_pose[self.gate_v][1]]]))
        dist_h_l = np.linalg.norm(gate_global_pose[0:2][:] - np.array([[self.gate_pose[self.gate_h_l][0]], [self.gate_pose[self.gate_h_l][1]]]))
        dist_h_r = np.linalg.norm(gate_global_pose[0:2][:] - np.array([[self.gate_pose[self.gate_h_r][0]], [self.gate_pose[self.gate_h_r][1]]]))
        ##

        ## Classify observed gate
        yaw = Rotation.from_quat([self.local_pose.pose.orientation.x, self.local_pose.pose.orientation.y, self.local_pose.pose.orientation.z, self.local_pose.pose.orientation.w]).as_euler('zyx', degrees=True)[0]
        print(yaw)
        if abs(yaw) < 80:
            if self.gate_first == self.gate_v:
                gate_seeing.data = 'vertical'
                print('1-1')
            else:
                if (dist_h_l > 1.0) and (dist_h_r > 1.0):
                    gate_seeing.data = 'unknown'
                    print('1-2-1')
                elif dist_h_l < dist_h_r:
                    gate_seeing.data = 'left'
                    print('1-2-2')
                else:
                    gate_seeing.data = 'right'
                    print('1-2-3')
        elif abs(yaw) > 100:
            if self.gate_first == self.gate_v:
                if (dist_h_l > 1.0) and (dist_h_r > 1.0):
                    gate_seeing.data = 'unknown'
                    print('2-1-1')
                elif dist_h_l < dist_h_r:
                    gate_seeing.data = 'left'
                    print('2-1-2')
                else:
                    gate_seeing.data = 'right'
                    print('2-1-3')
            else:
                gate_seeing.data = 'vertical'
                print('2-2')
        else:
            if (dist_v < 1.0 ) or (abs(gate_global_pose[2][0] - self.gate_pose[self.gate_v][2]) > 0.45):
                gate_seeing.data = 'vertical'
                print('3-1')
            elif abs(gate_global_pose[2][0] - self.gate_pose[self.gate_h_l][2]) < 0.25:
                if (dist_h_l > 1.0) and (dist_h_r > 1.0):
                    gate_seeing.data = 'unknown'
                    print('3-2-1')
                elif dist_h_l < dist_h_r:
                    gate_seeing.data = 'left'
                    print('3-2-2')
                else:
                    gate_seeing.data = 'right'
                    print('3-2-3')
            else:
                gate_seeing.data = 'unknown'
                print('3-3')

        if gate_seeing.data == 'vertical':
            self.gate_observing = self.gate_v
        elif gate_seeing.data == 'left':
            self.gate_observing = self.gate_h_l
        elif gate_seeing.data == 'right':
            self.gate_observing = self.gate_h_r
        elif gate_seeing.data == 'unknown':
            self.gate_observing = -1

        print(gate_seeing.data)
        ##

        # print("%.1f %.1f %.1f" % (dist_v, dist_h_l, dist_h_r))
        # print(self.gate_seeing.data)

        # Update gate position measurement
        self.z[2][0] = msg.pose.position.x
        self.z[3][0] = msg.pose.position.y

        # Publish classified gate
        self.gate_seeing_pub.publish(gate_seeing)


if __name__ == "__main__":
    try:
        ekf_slam = EKFSLAM()
        while not rospy.is_shutdown():
            ekf_slam.loop()

    except rospy.ROSInterruptException:
        pass

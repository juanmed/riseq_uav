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
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped


class EKFSLAM:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('riseq_estimation_EKF_SLAM')
        self.frequency = 10.0
        self.r = rospy.Rate(self.frequency)

        # Gate parameters
        self.gate_v = 0
        self.gate_h_l = 1
        self.gate_h_r = 2
        self.gate_detected = np.array([False, False, False])
        self.gate_pose = np.array([[0.0, 0.0, 1.9],
                                   [0.0, 0.0, 1.7],
                                   [0.0, 0.0, 1.7]])
        self.init_inf = 1e6
        self.gate_observing = -1

        self.cur_vo_pose = PoseStamped()
        self.last_vo_pose = PoseStamped()

        # State
        self.F = np.eye(10)
        self.x_pre = np.zeros((10, 1))
        self.x_est = np.array([[0.0],                               # drone real position
                               [0.0],
                               [0.0],                               # drone VO drift x
                               [0.0],                               # drone VO drift y
                               [self.gate_pose[self.gate_v][0]],    # vertical gate position in local frame
                               [self.gate_pose[self.gate_v][1]],
                               [self.gate_pose[self.gate_h_l][0]],  # horizontal left gate position in local frame
                               [self.gate_pose[self.gate_h_l][1]],
                               [self.gate_pose[self.gate_h_r][0]],  # horizontal right gate position in local frame
                               [self.gate_pose[self.gate_h_r][1]]])
        self.P_pre = np.zeros((10, 10))
        self.P_est = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, self.init_inf, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, self.init_inf, 0.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.init_inf, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.init_inf, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.init_inf, 0.0],
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.init_inf]])

        self.B = np.zeros((10, 2))
        self.B[0][0] = 1.0
        self.B[1][1] = 1.0
        self.u = np.zeros((2, 1))                                   # odometry(traveled distance) as input. B*u=I*(v*dt)
        self.Q = np.zeros((10, 10))
        self.Q[0:4, 0:4] = np.eye(4) * 1e-2

        self.z = np.array([[0.0],                                   # VO position. real position + drift
                           [0.0],
                           [0.0],                                   # distance to the gate in local frame
                           [0.0]])
        self.H_full = np.array([[1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                               [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                               [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                               [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        gate_cov = 0.5**2
        self.R = np.zeros((4, 4))
        self.R[0:2, 0:2] = np.eye(2) * 1e-2
        self.R[2:4, 2:4] = np.ones((2, 2)) * gate_cov

        # Publisher, Subscriber
        self.comp_pose_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.drift_pub = rospy.Publisher('/riseq/drone/vo_drift', PoseStamped, queue_size=10)

        rospy.Subscriber('/zed/zed_node/pose', PoseStamped, self.vo_pose_cb)
        rospy.Subscriber('/riseq/gate/lpf_global/global_pose', PoseStamped, self.gate_cb)

    def loop(self):
        # Update VO measurements
        self.z[0][0] = self.last_vo_pose.pose.position.x
        self.z[1][0] = self.last_vo_pose.pose.position.y
        self.u[0][0] = self.cur_vo_pose.pose.position.x - self.last_vo_pose.pose.position.x
        self.u[1][0] = self.cur_vo_pose.pose.position.y - self.last_vo_pose.pose.position.y
        self.last_vo_pose.pose.position.x = self.cur_vo_pose.pose.position.x
        self.last_vo_pose.pose.position.y = self.cur_vo_pose.pose.position.y

        # Update gate position measurement
        if self.gate_observing == self.gate_v:
            self.z[2][0] = self.gate_pose[self.gate_v][0] - self.x_est[0][0]
            self.z[3][0] = self.gate_pose[self.gate_v][1] - self.x_est[1][0]
        elif self.gate_observing == self.gate_h_l:
            self.z[2][0] = self.gate_pose[self.gate_h_l][0] - self.x_est[0][0]
            self.z[3][0] = self.gate_pose[self.gate_h_l][1] - self.x_est[1][0]
        elif self.gate_observing == self.gate_h_r:
            self.z[2][0] = self.gate_pose[self.gate_h_r][0] - self.x_est[0][0]
            self.z[3][0] = self.gate_pose[self.gate_h_r][1] - self.x_est[1][0]

        ## Kalman Filter
        # Prediction
        self.x_pre = np.dot(self.F, self.x_est) + np.dot(self.B, self.u)
        self.P_pre = np.linalg.multi_dot([self.F, self.P_est, self.F.T]) + self.Q

        # Correction, Update
        if self.gate_observing == -1:
            H = self.H_full[0:2][:]
            K = np.linalg.multi_dot([self.P_pre, H.T, np.linalg.inv(np.linalg.multi_dot([H, self.P_pre, H.T]) + self.R[0:2, 0:2])])
            self.x_est = self.x_pre + np.dot(K, self.z[0:2][0] - np.dot(H, self.x_pre))
        else:
            H = np.vstack([self.H_full[0:2, :], self.H_full[2*(self.gate_observing+1):2*(self.gate_observing+2), :]])
            K = np.linalg.multi_dot([self.P_pre, H.T, np.linalg.inv(np.linalg.multi_dot([H, self.P_pre, H.T]) + self.R)])
            self.x_est = self.x_pre + np.dot(K, self.z - np.dot(H, self.x_pre))
        self.P_est = np.dot(np.eye(10) - np.dot(K, H), self.P_pre)

        self.gate_observing = -1
        ##

        # Publish
        drift = PoseStamped()
        drift.header.stamp = rospy.Time.now()
        drift.pose.position.x = self.x_est[2][0]
        drift.pose.position.y = self.x_est[3][0]
        self.drift_pub.publish(drift)

        rospy.loginfo("Gate Positions")
        print(self.gate_pose)
        print("%.2f, %.2f" % (self.x_est[4][0], self.x_est[5][0]))
        print("%.2f, %.2f" % (self.x_est[6][0], self.x_est[7][0]))
        print("%.2f, %.2f" % (self.x_est[8][0], self.x_est[9][0]))

        self.r.sleep()

    def vo_pose_cb(self, msg):
        self.cur_vo_pose.header.stamp = rospy.Time.now()
        self.cur_vo_pose.pose.position.x = msg.pose.position.x
        self.cur_vo_pose.pose.position.y = msg.pose.position.y

        compensated_pose = PoseStamped()
        compensated_pose.header.stamp = msg.header.stamp
        compensated_pose.header.frame_id = msg.header.frame_id
        compensated_pose.pose.position.x = msg.pose.position.x - self.x_est[2][0]
        compensated_pose.pose.position.y = msg.pose.position.y - self.x_est[3][0]
        compensated_pose.pose.position.z = msg.pose.position.z
        compensated_pose.pose.orientation.x = msg.pose.orientation.x
        compensated_pose.pose.orientation.y = msg.pose.orientation.y
        compensated_pose.pose.orientation.z = msg.pose.orientation.z
        compensated_pose.pose.orientation.w = msg.pose.orientation.w
        self.comp_pose_pub.publish(compensated_pose)

    def gate_cb(self, msg):
        ## Register gate position
        if (self.gate_detected[self.gate_v] == False) and (abs(msg.pose.position.z - self.gate_pose[self.gate_v][2]) > 0.5):
        # Identify vertical gates by its height
            print(self.gate_detected)
            self.gate_detected[self.gate_v] = True
            print(self.gate_detected)
            
            self.x_est[4][0] = msg.pose.position.x
            self.x_est[5][0] = msg.pose.position.y

        elif (self.gate_detected[self.gate_h_l] == False) and (self.gate_detected[self.gate_h_r] == False):
        # Save the position temporally if both positions are uncertain
            print(self.gate_detected)
            self.gate_detected[self.gate_h_l] = True
            print(self.gate_detected)
            
            self.x_est[6][0] = msg.pose.position.x
            self.x_est[7][0] = msg.pose.position.y

        elif (self.gate_detected[self.gate_h_l] == True) and (self.gate_detected[self.gate_h_r] == False):
        # Compare two horizontal gates' position
            if np.linalg.norm(np.array([[self.gate_pose[self.gate_h_l][0]], [self.gate_pose[self.gate_h_l][1]]]) - np.array([[msg.pose.position.x], [msg.pose.position.y]])) > 1.0:
                print(self.gate_detected)
                self.gate_detected[self.gate_h_r] = True
                print(self.gate_detected)
                
                self.x_est[8][0] = msg.pose.position.x
                self.x_est[9][0] = msg.pose.position.y
        ##

        ## Classify detected gate
        # Calculate between measured data and gates in existence
        if self.gate_detected[self.gate_v] == True:
            distance_v = np.linalg.norm(np.array([[self.x_est[4][0]], [self.x_est[5][0]]]) - np.array([[msg.pose.position.x], [msg.pose.position.y]]))
        if self.gate_detected[self.gate_h_l] == True:
            distance_h_l = np.linalg.norm(np.array([[self.x_est[6][0]], [self.x_est[7][0]]]) - np.array([[msg.pose.position.x], [msg.pose.position.y]]))
        if self.gate_detected[self.gate_h_r] == True:
            distance_h_r = np.linalg.norm(np.array([[self.x_est[8][0]], [self.x_est[9][0]]]) - np.array([[msg.pose.position.x], [msg.pose.position.y]]))

        # Get the nearest gate
        if (self.gate_detected[self.gate_v] == True) and (self.gate_detected[self.gate_h_l] == True) and (self.gate_detected[self.gate_h_r] == True):
            if min(distance_v, distance_h_l, distance_h_r) == distance_v:
                self.gate_pose[self.gate_v][0] = msg.pose.position.x
                self.gate_pose[self.gate_v][1] = msg.pose.position.y
                self.gate_observing = self.gate_v
            elif min(distance_v, distance_h_l, distance_h_r) == distance_h_l:
                self.gate_pose[self.gate_h_l][0] = msg.pose.position.x
                self.gate_pose[self.gate_h_l][1] = msg.pose.position.y
                self.gate_observing = self.gate_h_l
            elif min(distance_v, distance_h_l, distance_h_r) == distance_h_r:
                self.gate_pose[self.gate_h_r][0] = msg.pose.position.x
                self.gate_pose[self.gate_h_r][1] = msg.pose.position.y
                self.gate_observing = self.gate_h_r
        elif (self.gate_detected[self.gate_v] == True) and (self.gate_detected[self.gate_h_l] == True):
            if min(distance_v, distance_h_l) == distance_v:
                self.gate_pose[self.gate_v][0] = msg.pose.position.x
                self.gate_pose[self.gate_v][1] = msg.pose.position.y
                self.gate_observing = self.gate_v
            elif min(distance_v, distance_h_l) == distance_h_l:
                self.gate_pose[self.gate_h_l][0] = msg.pose.position.x
                self.gate_pose[self.gate_h_l][1] = msg.pose.position.y
                self.gate_observing = self.gate_h_l
        elif self.gate_detected[self.gate_v] == True:
            self.gate_pose[self.gate_v][0] = msg.pose.position.x
            self.gate_pose[self.gate_v][1] = msg.pose.position.y
            self.gate_observing = self.gate_v
        elif self.gate_detected[self.gate_h_l] == True:
            self.gate_pose[self.gate_h_l][0] = msg.pose.position.x
            self.gate_pose[self.gate_h_l][1] = msg.pose.position.y
            self.gate_observing = self.gate_h_l
        ##

        ## Identify horizontal gates
        if (self.gate_detected[self.gate_v] == True) and (self.gate_detected[self.gate_h_l] == True) and (self.gate_detected[self.gate_h_r] == True):
            distance_l = np.linalg.norm(np.array([[self.x_est[4][0]], [self.x_est[5][0]]]) - np.array([[self.x_est[6][0]], [self.x_est[7][0]]]))
            distance_r = np.linalg.norm(np.array([[self.x_est[4][0]], [self.x_est[5][0]]]) - np.array([[self.x_est[8][0]], [self.x_est[9][0]]]))
            if distance_l > distance_r:
                tmp = self.x_est[6][0]
                self.x_est[6][0] = self.x_est[8][0]
                self.x_est[8][0] = tmp
                tmp = self.x_est[7][0]
                self.x_est[7][0] = self.x_est[9][0]
                self.x_est[9][0] = tmp
                tmp = self.gate_pose[self.gate_h_l][0]
                self.gate_pose[self.gate_h_l][0] = self.gate_pose[self.gate_h_r][0]
                self.gate_pose[self.gate_h_r][0] = tmp
                tmp = self.gate_pose[self.gate_h_r][0]
                self.gate_pose[self.gate_h_r][0] = self.gate_pose[self.gate_h_l][0]
                self.gate_pose[self.gate_h_l][0] = tmp

                if self.gate_observing == self.gate_h_l:
                    self.gate_observing = self.gate_h_r
                elif self.gate_observing == self.gate_h_r:
                    self.gate_observing = self.gate_h_l
        ##


if __name__ == "__main__":
    try:
        ekf_slam = EKFSLAM()
        while not rospy.is_shutdown():
            ekf_slam.loop()

    except rospy.ROSInterruptException:
        pass

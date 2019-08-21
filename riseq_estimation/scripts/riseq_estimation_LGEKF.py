#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.0.0
brief: This code is using Lie-group Extended Kalman Filter to estimate drones' states.

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

import lie

from std_msgs.msgs import Vector3
from sensor_msgs.msgs import Imu
from geometry_msgs import TwistStamped
from nav_msgs import Odometry


class LieGroupExtendedKalmanFilter():
    def __init__(self):
        rospy.init_node('riseq_estimation_LGEKF')

        self.frequency = 200
        self.r = rospy.Rate(self.frequency)
        self.dt = 1.0/self.frequency
        self.gravity = np.array([[0], [0], [9.807]])
        self.R_B_C = np.array([[0, 0, 1],               # Rotation matrix converts camera frame to body frame
                               [-1, 0, 0],
                               [0, -1, 0]])
        self.fusion1 = False                            # Sensor fusion checker. VO

        # State variables: T, w_B, v_B, a_B, scale
        # G = SE(3) x R3 x R3 x R3 x R1
        self.x_pre = np.eye((18, 18))
        self.x_est = np.eye((18, 18))
        
        # Initial VO scale
        self.x_pre[16][17] = 1
        self.x_est[16][17] = 1

        self.P_pre = np.eye(16) * 0.01
        self.P_est = np.eye(16) * 0.01

        # Measurement: gyro->w_B, accel->a_B, VO->w_B,v_B
        # G' = R3 x R3 x R3 x R3
        self.z = np.eye(16)

        self.R = np.eye(12) * 0.01
        self.R[0:3, 0:3] = np.eye(3) * 0.000001
        self.R[3:6, 3:6] = np.eye(3) * 0.0005
        self.Q = np.eye(16) * 0.001

        self.pub_state = rospy.Publisher('/riseq/estimation/state', Odometry, queue_size=10)
        self.pub_acceleration = rospy.Publisher('/riseq/estimation/linear_acceleration', Vector3, queue_size=10)

        rospy.Subscriber('/mavros/imu/data_raw', Imu, self.imuCb)
        rospy.Subscriber('/riseq/estimation/vo', TwistStamped, self.visualOdometryCb)


    def loop(self):
        self.x_pre, self.P_pre = self.predict(self.x_est, self.P_pre)
        self.x_est, self.P_est = self.update(self.x_pre, self.P_pre, self.z)

        self.pubTopic(self.x_est)

        if self.fusion1 == True:
            self.fusion1 = False
        self.r.sleep()


    def predict(self, x, P):
        if self.fusion1 == True:
            T_dot = np.zeros((4, 4))
            T_dot[0:3, 0:3] = lie.hat3(x[4:7, 7]*self.dt)                       #rotation matrix
            T_dot[0:3, 3] = x[8:11, 11]*self.dt + x[12:15, 15]*(self.dt**2)/2   #position

            V_dot = np.zeros((4, 4))
            V_dot[0:3, 3] = x[12:15, 15]*self.dt                                #linear velocity

            Ohmega = np.eye(18)
            Ohmega[0:4, 0:4] = T_dot
            Ohmega[8:12, 8:12] = V_dot
            exp_Ohmega = np.zeros((18, 18))                                     # exp_G(Ohmega)
            exp_Ohmega[0:4, 0:4] = lie.exp_se3(Ohmega[0:4, 0:4])
            exp_Ohmega[8:12, 8:12] = lie.exp_se3(Ohmega[8:12, 8:12])
            exp_mOhmega = np.zeros((18, 18))                                    # exp_G(-Ohmega)
            exp_mOhmega[0:4, 0:4] = lie.exp_se3(-Ohmega[0:4, 0:4])
            exp_mOhmega[8:12, 8:12] = lie.exp_se3(-Ohmega[8:12, 8:12])

            F = self.Adj(exp_mOhmega)                                           # F = Adj_G(exp_G(-Ohmega))
            Phi_Ohmega = np.eye(16) - self.adj(Ohmega)/2                               # Phi = I - adj_G(Ohmega)/2

            return np.dot(x, exp_Ohmega), np.linalg.multi_dot([F, P, F.T]) + np.linalg.multi_dot([Phi_Ohmega, self.Q, Phi_Ohmega.T])


    def update(self, x, P, z):
        y = self.getInnovation(x, z)
        H = self.getJacobianH(x)
        K = self.getKalmanGain(P, H)
        Ky = np.dot(K, y)

        if self.fusion1 == True:
            exp_Ky = np.eye(18)
            exp_Ky[0:4, 0:4] = lie.exp_se3(lie.hat6(Ky[0:3, 0], Ky[3:6, 0]))
            exp_Ky[4:7, 7] = Ky[6:9, 0]
            exp_Ky[8:11, 11] = Ky[9:12, 0]
            exp_Ky[12:15, 15] = Ky[12:15, 0]
            exp_Ky[16][17] = Ky[15][0]

            Phi_Ky = np.eye(16) - self.adj(Ky)

            return x, np.linalg.multi_dot([Phi_Ky, (np.eye(16) - np.dot(K, H)), P, Phi_Ky])


    def Adj(self, G):
        Adj_G = np.eye(16)
        Adj_G[0:7, 0:7] = lie.Adj_se3(G[0:4, 0:4])
        return Adj_G


    def adj(self, g):
        adj_g = np.zeros(16)
        adj_g[0:7, 0:7] = lie.adj_se3(lie.vee3(g[0:3, 0:3]), g[0:3, 3])
        return adj_g


    def getInnovation(self, x, z):
        if self.fusion1 == True:
            hx = np.eye(16)
            hx[0:3, 3] = x[4:7, 7]                                          # w_gyro (+ gyro bias)
            hx[4:7, 7] = x[12:15, 15] + np.dot(x[0:3, 0:3].T, self.gravity) # a_accel + gravity (+ accel bias)
            hx[8:11, 11] = x[4:7, 7]                                        # w_VO
            hx[12:15, 15] = x[8:11, 11]                                     # v_VO
            diff = np.dot(np.linalg.inv(hx), z)

            log_Gp = diff - np.eye(16)

            vee_Gp = np.zeros((12, 1))
            vee_Gp[0:3, 0] = log_Gp[0:3, 3]
            vee_Gp[3:6, 0] = log_Gp[4:7, 7]
            vee_Gp[6:9, 0] = log_Gp[8:11, 11]
            vee_Gp[9:12, 0] = log_Gp[12:15, 15]
            return vee_Gp


    def getJacobianH(self, x):
        #                 R        p_B      w_B      v_B      a_B      l
        return np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 1/x[16][17], 0, 0, 0, 0, 0, -x[8][11]/(x[16][17]**2)],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/x[16][17], 0, 0, 0, 0, -x[9][11]/(x[16][17]**2)],
                         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1/x[16][17], 0, 0, 0, -x[10][11]/(x[16][17]**2)]])


    def getKalmanGain(self, P, H):
        if self.fusion1 == True:
            S = np.linalg.multi_dot([H, P, H.T]) + self.R           # innovation covariance matrix
            return np.linalg.multi_dot([P, H.T, np.linalg.inv(S)])


    def pubTopic(self, x):
        state = Odometry()
        p_W = np.dot(x[0:3, 0:3].T, x[0:3, 3])
        state.pose.pose.position.x = p_W[0]
        state.pose.pose.position.y = p_W[1]
        state.pose.pose.position.z = p_W[2]
        state.pose.pose.orientation.w, state.pose.pose.orientation.x, state.pose.pose.orientation.y, state.pose.pose.orientation.z = lie.r2q(x[0:3, 0:3])
        v_W = np.dot(x[0:3, 0:3].T, x[8:11, 11])
        state.twist.twist.linear.x = v_W[0]
        state.twist.twist.linear.y = v_W[1]
        state.twist.twist.linear.z = v_W[2]
        state.twist.twist.angular.x = x[4][7]
        state.twist.twist.angular.y = x[5][7]
        state.twist.twist.angular.z = x[6][7]

        accel = Vector3()
        a_W = np.dot(x[0:3, 0:3].T, x[12:15, 15])
        accel.x = a_W[0]
        accel.y = a_W[1]
        accel.z = a_W[2]

        self.pub_state.publish(state)
        self.pub_acceleration(accel)


    def imuCb(self, msg):
        self.z[0][3] = msg.angular_velocity.x
        self.z[1][3] = msg.angular_velocity.y
        self.z[2][3] = msg.angular_velocity.z
        self.z[4][7] = msg.linear_acceleration.x
        self.z[5][7] = msg.linear_acceleration.y
        self.z[6][7] = msg.linear_acceleration.z


    def visualOdometryCb(self, msg):
        angular = np.array([[msg.angular.x], [msg.angular.y], [msg.angular.z]])
        self.z[8:11, 11] = np.dot(self.R_B_C, angular)

        linear = np.array([[msg.linear.x], [msg.linear.y], [msg.linear.z]])
        self.z[12:15, 15] = np.dot(self.R_B_C, linear)

        self.fusion1 = True
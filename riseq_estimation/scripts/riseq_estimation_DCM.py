#!/usr/bin/env python

import rospy
import numpy as np
from numpy.linalg import multi_dot, norm, inv
from scipy.signal import cont2discrete
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

from eugene_kinematics import hat, rotation2euler


class DirectionCosineMatrix():
    def imu_cb(self, imu):
        self.z1[0] = imu.linear_acceleration.x
        self.z1[1] = imu.linear_acceleration.y
        self.z1[2] = imu.linear_acceleration.z
        self.z1[3] = imu.angular_velocity.x
        self.z1[4] = imu.angular_velocity.y
        self.z1[5] = imu.angular_velocity.z


    def input_cb(self, U):
        self.u[0] = 0
        self.u[1] = 0
        self.u[2] = 0
        self.u[3] = 0


    def __init__(self):
        rospy.init_node('riseq_estimation_DCM')

        self.rate = 200
        self.r = rospy.Rate(self.rate)
        self.dT = 1.0/self.rate
        self.g = 9.81

        self.mass = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_mass')
        self.Ixx = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_inertia_xx')
        self.Iyy = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_inertia_yy')
        self.Izz = rospy.get_param('/uav/flightgoggles_uav_dynamics/vehicle_inertia_zz')
        self.arm_length = rospy.get_param('/uav/flightgoggles_uav_dynamics/moment_arm')
        self.k_thrust = rospy.get_param('/uav/flightgoggles_uav_dynamics/thrust_coefficient')
        self.k_torque = rospy.get_param('/uav/flightgoggles_uav_dynamics/torque_coefficient')
        self.drag = rospy.get_param('/uav/flightgoggles_uav_dynamics/drag_coefficient')

        self.gyro_var = rospy.get_param('/uav/flightgoggles_imu/gyroscope_variance')
        self.accel_var = rospy.get_param('/uav/flightgoggles_imu/accelerometer_variance')
        self.range_var = rospy.get_param('/uav/flightgoggles_laser/rangefinder_variance')
        self.init_pose = rospy.get_param('/uav/flightgoggles_uav_dynamics/init_pose')

        self.attitude_pub = rospy.Publisher('/dcm/attitude', Vector3, queue_size=10)
        self.attitude = Vector3()
        rospy.Subscriber('/uav/sensors/imu', Imu, self.imu_cb)

        self.F1 = np.zeros((6, 6))
        self.F1d = np.zeros((6, 6))
        self.B1 = np.zeros((6, 4))
        self.B1d = np.zeros((6, 4))
        self.H1 = np.array([[self.g, 0, 0, 0, 0, 0],
                            [0, self.g, 0, 0, 0, 0],
                            [0, 0, self.g, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        self.D1 = np.zeros((6, 4))

        self.x1_est = np.array([[0], [0], [1], [0], [0], [0]])  # The third column of the rotation matrix, 3 gyro biases
        self.x1_pre = np.zeros((6, 1))
        self.z1 = np.zeros((6, 1))  # accelerometer, gyro

        self.P1_pre = np.eye(6) * 0.01
        self.P1_est = np.eye(6) * 0.01
        self.Q1 = np.zeros((6, 6))
        self.R1 = np.array([[self.accel_var, 0, 0, 0, 0, 0],
                            [0, self.accel_var, 0, 0, 0, 0],
                            [0, 0, self.accel_var, 0, 0, 0],
                            [0, 0, 0, self.gyro_var, 0, 0],
                            [0, 0, 0, 0, self.gyro_var, 0],
                            [0, 0, 0, 0, 0, self.gyro_var]])

        self.F2 = np.zeros((6, 6))
        self.F2d = np.zeros((6, 6))
        self.B2 = np.zeros((6, 4))
        self.B2d = np.zeros((6, 4))
        self.H2 = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        self.D2 = np.zeros((6, 4))

        self.x2_est = np.array([[0], [1], [0], [0], [0], [0]])  # The second column of the rotation matrix, 3 gyro biases
        self.x2_pre = np.zeros((6, 1))
        self.z2 = np.zeros((6, 1))  # magnetometer, gyro

        self.P2_pre = np.eye(6) * 0.001
        self.P2_est = np.eye(6) * 0.001
        self.Q2 = np.zeros((6, 6))
        self.R2 = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, self.gyro_var, 0, 0],
                            [0, 0, 0, 0, self.gyro_var, 0],
                            [0, 0, 0, 0, 0, self.gyro_var]])

        self.F2 = np.zeros((6, 6))
        self.F2d = np.zeros((6, 6))
        self.B2 = np.zeros((6, 4))
        self.B2d = np.zeros((6, 4))
        self.H2 = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])
        self.D2 = np.zeros((6, 4))

        self.u = np.zeros((4, 1))

    def loop(self):
        # Attitude estimation
        self.F1[0:3, 3:6] = hat(self.x1_est[0:3, :])
        (self.F1d, self.B1d, self.H1, self.D1, self.dT) = cont2discrete((self.F1, self.B1, self.H1, self.D1), self.dT)
        # predict
        self.x1_pre = np.dot(self.F1d, self.x1_est) + np.dot(self.B1d, self.u)
        self.P1_pre = multi_dot([self.F1d, self.P1_est, self.F1d.T]) + self.Q1

        # Kalman gain
        K1 = multi_dot([self.P1_pre, self.H1.T, inv(multi_dot([self.H1, self.P1_pre, self.H1.T]) + self.R1)])

        # update
        y1 = self.z1 - np.dot(self.H1, self.x1_pre)
        self.x1_est = self.x1_pre + np.dot(K1, y1)
        self.x1_est[0:3, :] = self.x1_est[0:3, :] / norm(self.x1_est[0:3, :])   # normalization
        self.P1_est = np.dot(np.eye(6) - np.dot(K1, self.H1), self.P1_pre)

        # Orientation estimation
        self.F2[0:3, 3:6] = hat(self.x2_est[0:3, :])
        (self.F2d, self.B2d, self.H2, self.D2, self.dT) = cont2discrete((self.F2, self.B2, self.H2, self.D2), self.dT)
        # predict
        self.x2_pre = np.dot(self.F2d, self.x2_est) + np.dot(self.B2d, self.u)
        self.P2_pre = multi_dot([self.F2d, self.P2_est, self.F2d.T]) + self.Q2

        # Kalman gain
        K2 = multi_dot([self.P2_pre, self.H2.T, inv(multi_dot([self.H2, self.P2_pre, self.H2.T]) + self.R2)])

        # update
        y2 = self.z2 - np.dot(self.H2, self.x2_pre)
        self.x2_est = self.x2_pre + np.dot(K2, y2)
        self.x2_est[0:3, :] = self.x2_est[0:3, :] / norm(self.x2_est[0:3, :])   # normalization
        self.P2_est = np.dot(np.eye(6) - np.dot(K2, self.H2), self.P2_pre)
        
        '''
        self.F[0:3, 6:9] = hat(self.x_est[0:3, :])
        self.F[0:3, 9:12] = -hat(self.x_est[0:3, :])
        self.F[3:6, 6:9] = hat(self.x_est[3:6, :])
        self.F[3:6, 9:12] = -hat(self.x_est[3:6, :])
        '''

        # Direct Cosine Matrix
        C = np.array([np.cross(self.x1_est[0:3, :].T, self.x2_est[0:3, :].T)[0],
                      self.x2_est[0:3, :].T[0],
                      self.x1_est[0:3, :].T[0]])
        (phi, theta, psi) = rotation2euler(C)   # convert to Euler angles

        # publish state
        self.attitude.x = phi
        self.attitude.y = theta
        self.attitude.z = psi
        self.attitude_pub.publish(self.attitude)

        self.r.sleep()

'''
if __name__ == "__main__":
    try:
        estimator = DirectionCosineMatrix()
        while not rospy.is_shutdown():
            estimator.loop()
    except rospy.ROSInterruptException:
        pass
'''
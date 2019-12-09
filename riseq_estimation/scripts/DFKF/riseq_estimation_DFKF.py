#!/usr/bin/env python

import rospy
from KalmanFilter import KalmanFilter as KF
from scipy.signal import cont2discrete

from sensor_msgs.msg import Imu
from riseq_control.msg import riseq_high_level_control, riseq_low_level_control



class DFKF():
    """Differential flatness based Kalman Filter for Quadrotor"""

    At = np.array([[0., 1., 0.],
                   [0., 0., 1.],
                   [0., 0., 0.]])
    Bt = np.array([[0.],
                   [0.],
                   [1.]])
    Ht = np.array([0., 0., 1.])
    Dt = np.array([0])

    def __init__(self, dt = 0.1, acc_var = 1e-3, gyro_var = 1e-3):
        
        self.dt = dt
        self.acc_var = acc_var
        self.gyro_var = gyro_var


    def init_filters(self):
        F, G, H, _, dt = cont2discrete((At, Bt, Ht, Dt), self.dt)
        Q = np.zeros((3,3))
        R = np.array([self.acc_var])
        self.KFx = KF(F,G,H,)
    
    def predict(self, u):
        self.KFx.predict(u)

    def update(self, z):
        self.KFx.update(z)

    def filter(self, u, z):
        self.predict(u)
        self.update(z)

    def inverse_map(self, )



class DF_Estimator():
    """docstring for DF_Estimator"""
    def __init__(self, dt = 0.03):
        self.dt = dt

        self.init_estimator()


    def init_estimator(self):

        self.imu_covariance_set = False
        self.imu_accel_covariance = 1e-3
        self.imu_gyro_covariance = 1e-3

        rospy.Subscriber('/pelican/imu', Imu, self.imu_cb)
        rospy.Subscriber('riseq/control/uav_high_level_control', riseq_high_level_control, self.hlc_cb)
        

        self.g = rospy.get_param("riseq/gravity", 9.81)
        self.mass = rospy.get_param("riseq/mass", 1.0)
        self.e1 = np.array([[1],[0],[0]])       # Vectors e1, e2, e3 generate R3
        self.e2 = np.array([[0],[1],[0]])
        self.e3 = np.array([[0],[0],[1]])

    def imu_cb(self, msg):
        if not self.imu_covariance_set:
            self.imu_covariance_set = True
            self.imu_accel_covariance = msg.linear_acceleration_covariance[0]
            self.imu_gyro_covariance = msg.angular_velocity_covariance[0]

        self.z = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        self.z = np.dot(self.Rbw,self.z.T) + self.g*self.e3

    def hlc_cb(self, msg):
        """
        High level controller input callback
        """
        self.Rbw = np.array(msg.rot).reshape(3,3)
        self.u = self.g*self.e3 + (msg.thrust.z/self.mass)*np.dot(self.Rbw,self.e3) 

    def 



        

if __name__ == '__main__':
    try:

    except rospy.ROSInterruptException:
     rospy.loginfo('ROS Terminated')
     pass 
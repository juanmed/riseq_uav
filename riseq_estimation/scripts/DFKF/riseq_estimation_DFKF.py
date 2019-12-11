#!/usr/bin/env python

import rospy
from KalmanFilter import KalmanFilter as KF
from scipy.signal import cont2discrete
import numpy as np

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from riseq_control.msg import riseq_high_level_control, riseq_low_level_control
from nav_msgs.msg import Odometry
from riseq_trajectory.msg import riseq_uav_trajectory



class DFKF():
    """Differential flatness based Kalman Filter for Quadrotor"""

    At = np.array([[0., 1., 0.],
                   [0., 0., 1.],
                   [0., 0., 0.]])
    Bt = np.array([[0.],
                   [0.],
                   [1.]])
    Ht = np.array([[1., 0., 0.],
                   [0., 0., 1.]])
    Dt = np.array([0])
    PROCESS_NOISE = .01

    def __init__(self, x0, dt = 0.1, acc_var = 1e-3, gyro_var = 1e-3, pose_var = 1e-3):
        
        self.dt = dt
        self.acc_var = acc_var
        self.gyro_var = gyro_var
        self.pose_var = pose_var
        self.x0 = x0

        self.init_filters()


    def init_filters(self):
        F, G, H, D, dt = cont2discrete((self.At, self.Bt, self.Ht, self.Dt), self.dt)
        Q = np.eye(3)*self.PROCESS_NOISE
        R = np.diag([self.pose_var, self.acc_var])
        print(R)
        self.KFx = KF(F,G,H,Q,R,x0 = np.zeros((3,1)))
        self.KFy = KF(F,G,H,Q,R,x0 = np.zeros((3,1)))
        self.KFz = KF(F,G,H,Q,R,x0 = np.zeros((3,1)))

    def predict(self, u):
        self.KFx.predict(u[0][0])
        self.KFy.predict(u[1][0])
        self.KFz.predict(u[2][0])

    def update(self, z):
        self.KFx.update(z[0])
        self.KFy.update(z[1])
        self.KFz.update(z[2])

    def filter(self, u, z):
        self.predict(u)
        self.update(z)
        return self.KFx.x, self.KFy.x, self.KFz.x




class DF_Estimator():
    """docstring for DF_Estimator"""
    def __init__(self, dt = 0.05):
        self.dt = dt

        self.init_estimator()


    def init_estimator(self):

        self.g = rospy.get_param("riseq/gravity", 9.81)
        self.mass = rospy.get_param("riseq/mass", 1.0)
        self.e1 = np.array([[1],[0],[0]])       # Vectors e1, e2, e3 generate R3
        self.e2 = np.array([[0],[1],[0]])
        self.e3 = np.array([[0],[0],[1]])

        # initial input and measurement
        self.u = np.zeros((3,1))
        self.Rbw = np.eye(3)
        self.a_m = np.zeros((3,1))

        self.imu_covariance_set = False
        self.pose_covariance_set = False
        rospy.Subscriber('/pelican/imu1', Imu, self.imu_cb)
        rospy.Subscriber('riseq/control/uav_high_level_control', riseq_high_level_control, self.hlc_cb)
        rospy.Subscriber('/pelican/pose_sensor1/pose_with_covariance', PoseWithCovarianceStamped, self.pose_sensor_cb)
        rospy.Subscriber('riseq/uav_trajectory', riseq_uav_trajectory, self.trajectory_cb)
        #rospy.Subscriber('/riseq/snap_sensor', )
        self.state_estimate_publisher = rospy.Publisher('/riseq/estimation/uav_dfkf_state', Odometry, queue_size = 10)


        while not self.imu_covariance_set:
            continue

        while not self.pose_covariance_set:
            continue        

        self.estimator = DFKF(dt = self.dt, acc_var = self.imu_accel_covariance , gyro_var = self.imu_gyro_covariance, pose_var = self.pose_covariance, x0 = 0)

        self.filter_timer = rospy.Timer(rospy.Duration(self.dt), self.filter)

    def imu_cb(self, msg):
        if not self.imu_covariance_set:
            self.imu_covariance_set = True
            self.imu_accel_covariance = msg.linear_acceleration_covariance[0]
            self.imu_gyro_covariance = msg.angular_velocity_covariance[0]

        self.a_m = np.array([[msg.linear_acceleration.x],[msg.linear_acceleration.y],[msg.linear_acceleration.z]])
        self.a_m = np.dot(self.Rbw,self.a_m) - self.g*self.e3
        #print(self.a_m)

    def pose_sensor_cb(self,msg):
        if not self.pose_covariance_set:
            self.pose_covariance_set = True
            self.pose_covariance = msg.pose.covariance[0]

        self.p_m = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def hlc_cb(self, msg):
        """
        High level controller input callback
        """
        self.Rbw = np.array(msg.rot).reshape(3,3)
        self.u = -self.g*self.e3 + (msg.thrust.z/self.mass)*np.dot(self.Rbw,self.e3) 
        #print(self.u)

    def snap_sensor_cb(self, msg):
        """
        """
        return 0

    def trajectory_cb(self, msg):

        self.acc = msg.acc.x 
        self.acc = msg.acc.y 
        self.acc = msg.acc.z 

        self.jerk = msg.jerk.x 
        self.jerk = msg.jerk.y 
        self.jerk = msg.jerk.z 

        self.jerk = msg.snap.x 
        self.jerk = msg.snap.y 
        self.jerk = msg.snap.z 

    def filter(self, timer):
        #print(self.u,self.a_m)

        z = []
        for i in range(3):
            z.append(np.array([[self.p_m[i]],[self.a_m[i][0]]])) # assemble measurement vector

        #self.u = np.zeros((3,1))
        x_dyn, y_dyn, z_dyn =self.estimator.filter(self.u, z)

        estimate = Odometry()
        estimate.header.stamp = rospy.Time.now()
        estimate.header.frame_id = ""
        estimate.pose.pose.position.x = x_dyn[0][0]
        estimate.pose.pose.position.y = y_dyn[0][0]
        estimate.pose.pose.position.z = z_dyn[0][0]

        estimate.twist.twist.linear.x = x_dyn[1][0]
        estimate.twist.twist.linear.y = y_dyn[1][0]
        estimate.twist.twist.linear.z = z_dyn[1][0]

        self.state_estimate_publisher.publish(estimate)
        
    def inverse_map(self, a):
        return 0
        

if __name__ == '__main__':
    try:
        rospy.init_node('riseq_dfkf_estimator', anonymous = True)
        estimation_node = DF_Estimator()

        rospy.loginfo(' RISEQ Differential Flatness Kalman Filter Started! ')
        rospy.spin()
        rospy.loginfo(' RISEQ Differential Flatness Kalman Filter Terminated ') 

    except rospy.ROSInterruptException:
     rospy.loginfo('ROS Terminated')
     pass 
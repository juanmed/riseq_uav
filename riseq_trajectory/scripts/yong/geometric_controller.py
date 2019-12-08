#################################
# [1] Geometric Tracking Control of a Quadrotor UAV on SE(3)
#
# [2] Control of Complex Manuevers for a Quadrotor UAV using Geometric Methods on SE(33
#################################

#!/usr/bin/env python

import rospy
import numpy as np
import tf
from riseq_trajectory.msg import riseq_uav_trajectory
from riseq_common.msg import riseq_uav_state
from mav_msgs.msg import Actuators


class Controller:
    """
    Reference in [1]
    f = (kx*ex + kv*ev +mge3 - m*a_des)(Re3)
    M = -kR*eR - kwew + w X Jw - J(wR.TR_dw_d - R.TR_da_d)

    J : inertia of moment
    eR = 1/2 * (R_d.TR - R.TR_d)v
    ew = w - R.TR_dw_d
    """
    def __init__(self):
        model_name = rospy.get_param("model_name", "iris")

        # Initialize drone's parameter
        # Iris in Rotors
        Ixx = rospy.get_param("riseq/Ixx", 0.0347563)
        Iyy = rospy.get_param("riseq/Iyy", 0.0458929)
        Izz = rospy.get_param("riseq/Izz", 0.0977)
        self.Inertia = np.diag([Ixx, Iyy, Izz])

        # /rotors_simulator/rotors_gazebo/resource/iris.yaml
        self.m = 1.52   # [kg]
        rotor_drag_coefficient = 8.06428e-05  # rotor_force_constant
        rolling_moment_coefficient = 0.000001  # rotor_moment_constant
        arm_length_front_x = 0.13  # [m]
        arm_length_back_x = 0.13
        arm_length_front_y = 0.22
        arm_length_back_y = 0.2
        max_rot_velocity = 838  # [rad/s]
        self.max_thrust = 4 * rotor_drag_coefficient * max_rot_velocity

        # r0 : front_right, r1 : back_left, r2 : front_left, r3 : back_right
        # U[4x1 matrix] : Input vector
        # R[4X1 matrix] : square of Rotor vector
        # U = K * R   ->    R = K-1 * U
        # u1[thrust] = k_d*r0^2 + k_d*r1^2 + k_d*r2^2 + k_d*r3^2
        # u2[roll] = -k_d*arm_length_front_y*r0^2 + k_d*arm_length_back_y*r1^2 + k_d*arm_length_front_y*r2^2 - k_d*arm_length_back_y*r3^2
        # u3[pitch] = -k_d*arm_length_front_x*r0_2 + k_d*arm_length_back_x*r1^2 - k_d*arm_length_front_x*r2^2 + k_d*arm_length_back_y*r3^2
        # u4[yaw] = -k_m*r0^2 -k_m*r1^2 + k_m*r2^2 + k_m*r3^2

        k_d = rotor_drag_coefficient
        k_m = rolling_moment_coefficient
        K = np.array([[k_d, k_d, k_d, k_d],
                    [-k_d*arm_length_front_y, k_d*arm_length_back_y, k_d*arm_length_front_y, -k_d*arm_length_back_y],
                    [-k_d*arm_length_front_x, k_d*arm_length_back_x, -k_d*arm_length_front_x, k_d*arm_length_back_y],
                    [-k_m, -k_m, k_m, k_m]])
        self.Kinv = np.linalg.inv(K)

        # Set control gain
        self.kx = np.array([1, 1, 1])
        self.kv = np.array([1, 1, 1])
        self.kR = np.diag([1, 1, 1])
        self.kw = np.diag([1, 1, 1])

        # Create subscriber and publisher
        rospy.Subscriber("riseq/trajectory/uav_trajectory", riseq_uav_trajectory, self.traj_cb)
        rospy.Subscriber("riseq/estimator/uav_estimated_state", riseq_uav_state, self.state_cb)
        self.actuators_pub = rospy.Publisher("/" + model_name + "/command/motor_speed", Actuators, queue_size=10)

    def traj_cb(self, msg):
        self.pos_des = msg.pose.position
        self.vel_des = msg.twist.linear
        self.acc_des = msg.acc

        # world to body reference transformation
        bRw_des = np.array(
            [[msg.rot[0], msg.rot[1], msg.rot[2]],
             [msg.rot[3], msg.rot[4], msg.rot[5]],
             [msg.rot[6], msg.rot[7], msg.rot[8]]])
        # body to world transformation
        self.R_des = bRw_des.T
        self.R_des = self.R_des
        self.w_des = msg.twist.angular
        self.w_dot_des = msg.ub

    def state_cb(self, msg):
        self.pos = msg.pose.position
        self.vel = msg.twist.linear

        # world to body transformation
        ori_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w]
        bRw = tf.transformations.quaternion_matrix(ori_quat)
        # body to world transformation
        self.R = bRw.T
        self.R = np.matrix(self.R)
        self.w = msg.twist.angular

    def pub_llc(self):
        pos_err = self.pos_des - self.pos
        vel_err = self.vel_des - self.vel

        # acc_des : feedforward, error : feedback
        g = 9.81
        e3 = np.array([0, 0, 1])
        t = self.kx * pos_err + self.kv * vel_err + self.m * g * e3 - self.m * self.acc_des

        # Desired Thrust
        F = t * self.R *e3
        if F < 0:
            F = 0
        elif F > self.max_thrust:
            F = self.max_thrust


        rot_err = 1/2 * self.R_des.T * self.R - self.R.T * self.R_des
        # v map from 3x3 skew-symmetric to 3x1 vector
        eR = np.array([0, 0, 0])
        eR[0] = -rot_err[1][2]  # x
        eR[1] = rot_err[0][2]   # y
        eR[2] = -rot_err[0][1]  # z

        ew = self.w - self.R.T * self.R_des * self.w_des
        # h map from 3x1 vector to 3x3 skew-symmetric
        w_h = np.array([[0, -self.w[2], self.w[1]],
                       [self.w[2], 0, -self.w[0]],
                       [-self.w[1], self.w[0], 0]])
        M = -self.kR * eR - self.kw * eR + np.cross(self.w, self.Inertia * self.w) - self.Inertia * (w_h * self.R.T * self.R_des * self.w_des - self.R.T * self.R_des * self.w_dot_des)
        U = np.array([F, M[0], M[1], M[2]])
        R2 = self.Kinv * U
        R = np.array([np.sqrt(R2[0]), np.sqrt(R2[1]), np.sqrt(R2[2]), np.sqrt(R2[3])])

        actuators = Actuators()
        actuators.angular_velocities = R
        actuators.header.stamp = rospy.Time.now()
        self.actuators_pub.publish(actuators)


if __name__ == "__main__":
    # Init Node
    rospy.init_node('riseq_controller', anonymous=True)

    # Make Class
    controller = Controller()

    try:
        rospy.loginfo("UAV Controller created")
        controller.pub_input()
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
        pass
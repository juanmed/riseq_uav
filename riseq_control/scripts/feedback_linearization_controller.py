"""
author:  Juan Medrano
version: 0.1
brief: A controller for a quadrotor. Based on 
        feedback linearization control theory and using euler angle representation
        for control of orientation.

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

import tf
import numpy as np
import control_gains as gains
import riseq_common.dyn_utils as utils
import riseq_common.physical_parameters as params
import riseq_tests.df_flat as df_flat
   
class Feedback_Linearization_Controller():

    def __init__(self, mass= 1.0, max_thrust = 1.0, min_thrust = 0.0, inertia = np.diag([1.0,1.0,1.0])):
        """
        Initialize this controller's parameters

        Args:
            mass (float): mass of the rigid body in kilograms
            max_thrust (float): maximum value of collective thrust
            min_thrust (float): minimum value of collective thrust
            inertia (3x3 np.array): inertia tensor of the quadrotor
        """
        self.mass = mass
        self.Inertia = inertia
        self.Traw = 0
        self.pos_error_integral = 0

        # PID gains for position error and body rate error controller 
        self.Kp = np.diag([gains.Kpx2, gains.Kpy2, gains.Kpz2])
        self.Kd = np.diag([gains.Kdx2, gains.Kdy2, gains.Kdz2])
        self.Ki = np.diag([gains.Kix2, gains.Kiy2, gains.Kiz2])

        self.dpr = np.array([-8.0]) 
        self.Kr, self.N_ur, self.N_xr = gains.calculate_pp_gains(gains.Ar, gains.Br, gains.Cr, gains.D_, self.dpr)
        self.Kr = self.Kr.item(0,0)

        self.max_thrust = max_thrust
        self.min_thrust = min_thrust

    def position_controller(self, state, ref):
        """
        Calculate required thrust, body orientation, and body rate in order to converge
        the state into the reference state:

        The order of elements for both the state and reference state is as follows:
        element 0: position (3x1 np.array)
        element 1: velocity (3x1 np.array)
        element 2: acceleration (3x1 np.array)
        element 3: jerk (3x1 np.array)
        element 4: snap (3x1 np.array)
        element 5: orientation rotation matrix (3x3 np.array)
        element 6: yaw angle (float)
        element 7: yaw angle rate (float)
        element 8: yaw angle acceleration (float)
        element 9: body rate in euler representation (3x1 np.array)

        Args:
            state (list): current state vector of the quadrotor
            ref (list): reference state vector of the quadrotor

        Returns:
            Required collective thrust (float), orientation (3x3 rotation matrix, np.array)
            and body rate (3x1 np.array)

        """
        p = state[0]
        v = state[1]
        Rbw = state[5]

        p_ref = ref[0]
        v_ref = ref[1]
        a_ref = ref[2]
        #j_ref = ref[3]
        #s_ref = ref[4]
        Rbw_ref = ref[5]
        yawdot_r = ref[7]
        euler_dot_ref = ref[9]

        # Calculate  PID control law for acceleration error: 
        self.pos_error_integral = self.pos_error_integral + (p - p_ref)
        self.a_e = -1.0*np.dot(self.Kp,p-p_ref) -1.0*np.dot(self.Kd,v-v_ref) -1.0*np.dot(self.Ki,self.pos_error_integral)  # PID control law

        # anti-windup integral control
        if((self.Traw >= self.max_thrust) or (self.Traw <= self.min_thrust)):
            Ki = np.diag([0.0, 0.0, 0.0])
            self.a_e2 = -1.0*np.dot(self.Kp,p-p_ref) -1.0*np.dot(self.Kd,v-v_ref) -1.0*np.dot(Ki,self.pos_error_integral)  # PID control law
        else:
            self.a_e2 = self.a_e            


        # Desired acceleration and thrust
        a_des = self.a_e2 + a_ref + params.g*params.e3
        wzb = np.dot(Rbw, params.e3)          # body z-axis expressed in world frame
        self.Traw = self.mass*np.dot(wzb.T,a_des)[0][0]            
        T = utils.saturate_scalar_minmax(self.Traw, self.max_thrust, self.min_thrust)    # Maximum possible thrust

        # Desired attitude
        zb_des = a_des / np.linalg.norm(a_des)
        Rbw_des, psi_ref = self.desired_attitude(zb_des, Rbw_ref)

        # Desired body rate
        Rbw_des_h = utils.to_homogeneous_transform(Rbw_des)
        psi_des, theta_des, phi_des = tf.transformations.euler_from_matrix(Rbw_des_h, axes = 'rzyx')
        euler_des = np.array([[phi_des],[theta_des],[psi_des]])
        
        Rbw_h = utils.to_homogeneous_transform(Rbw)
        psi, theta, phi = tf.transformations.euler_from_matrix(Rbw_h, axes = 'rzyx')
        euler = np.array([[phi],[theta],[psi]])
        
        w_des = self.desired_body_rate(euler, euler_des, euler_dot_ref, self.Kr)

        return T, Rbw_des, w_des

    def desired_attitude(self, zb_des, Rbw_ref):
        """
        Calculate a desired attitude based on a desired direction of body z-axis vector
        and a reference orientation

        Args:
            zb_des (3x1 np.array): desired body zb axis
            Rbw_ref (3x3 np.array): reference 3x3 orientation as rotation matrix

        Returns:
            Desired attitude as a 3x3 rotation matrix (np.array) and reference yaw angle
        """
        Rbw_h = utils.to_homogeneous_transform(Rbw_ref)
        psi_ref, theta_ref, phi_ref = tf.transformations.euler_from_matrix(Rbw_h, axes = 'rzyx')

        yc_des = np.array(df_flat.get_yc(psi_ref))
        xb_des = np.cross(yc_des, zb_des, axis = 0)
        xb_des = xb_des/np.linalg.norm(xb_des)
        yb_des = np.cross(zb_des, xb_des, axis = 0)

        Rbw_des = np.concatenate((xb_des, yb_des, zb_des), axis = 1)
        return Rbw_des, psi_ref

    def desired_body_rate(self, euler, euler_ref, euler_dot_ref, gain = 1.0):
        """
        Compute the body rate that will take the orientation of the quadrotor (euler) to 
        the reference orientation (euler_ref) using euler_dot_ref as feedforward term.
        Control law is of the form: u = K*(euler_ref - euler)
    
        Args:
            euler (3x1 np.array): current orientation of the quadrotor in euler angles (phi -roll-, theta -pitch-, psi -yaw-)
            euler_ref (3x1 np.array): reference orientation of the quadrotor in euler angles (phi -roll-, theta -pitch-, psi -yaw-)
            euler_dot_ref (3x1 np.array): reference body rate expressed in euler angle rates
            gain (float): control gain, must be positive value

        Returns:
            Desired body angular velocity 3x1 np.array

        """
        gain_matrix = np.diag([gain, gain, gain])
        euler_error = euler - euler_ref
        u = -1.0*np.dot(gain_matrix, euler_error)
        
        euler_dot = u + euler_dot_ref

        # compute w_b angular velocity commands as
        #  w_b = K.inv * uc
        #  where  (euler dot) = K*(angular_velocity)
        #  K is -not- a gain matrix, see definition below
        phi = euler[0][0]
        theta = euler[1][0]
        psi = euler[2][0]
        K = np.array([[1.0, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                      [0.0, np.cos(phi), -1.0*np.sin(phi)], 
                      [0.0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

        Kinv = np.linalg.inv(K)        

        w_des = np.dot(Kinv, euler_dot)

        return w_des

    def attitude_controller(self, angular_velocity, angular_velocity_des, angular_velocity_dot_ref, gain = 8.0):
        """
        Compute control torque vector based on current angular velocity, desired angular velocity
        and a reference angular acceleration. This controller was designed using feedback linearization
        of the rotation dynamics of drone motion.

        Based on:
          Mclain, T., Beard, R. W., Mclain, T. ;, Beard, R. W. ;, Leishman, R. C.
          Differential Flatness Based Control of a Rotorcraft For Aggressive Maneuvers 
          (September), 2688-2693.

        Args:
            angular_velocity (3x1 np.array): current body angular velocity
            angular_velocity_des (3x1 np.array): desired body angular velocity
            angular_velocity_dot_ref (3x1 np.array): reference body angular acceleration
            gain (float): controller gain

        Returns:  
            3x1 np.array control torque
        """

        # angular velocity error
        w_e = angular_velocity - angular_velocity_des

        # control input ub_e for angular velocity error 
        gain_matrix = np.diag([gain, gain, gain])
        ub_e = -1.0*np.dot(gain_matrix, w_e)

        # control input ub for angular velocity
        ub = ub_e + angular_velocity_dot_ref

        # control torque M
        M = np.dot(self.Inertia, ub) + np.cross(angular_velocity, np.dot(self.Inertia, angular_velocity), axis = 0)

        return M
import tf
import numpy as np
import control_gains as gains
import riseq_common.dyn_utils as utils
import riseq_common.physical_parameters as params
import riseq_tests.df_flat as df_flat


class Geometric_Controller():

    def __init__(self, mass= 1.0, max_thrust = 1.0, min_thrust = 0.0):

        self.mass = mass
        self.Traw = 0
        self.Fd_dot = 0
        self.pos_error_integral = 0

        # PID gains for position error controller 
        self.Kp = np.diag([gains.Kpx2, gains.Kpy2, gains.Kpz2])
        self.Kd = np.diag([gains.Kdx2, gains.Kdy2, gains.Kdz2])
        self.Ki = np.diag([gains.Kix2, gains.Kiy2, gains.Kiz2])

        self.max_thrust = max_thrust
        self.min_thrust = min_thrust

    def position_controller(self, state, ref):
        """
        """
        p = state[0]
        v = state[1]
        Rbw = state[5]

        p_ref = ref[0]
        v_ref = ref[1]
        a_ref = ref[2]
        j_ref = ref[3]
        #s_ref = ref[4]
        Rbw_ref = ref[5]
        yawdot_ref = ref[7]

        # Calculate  PID control law for acceleration error: 
        self.pos_error_integral = self.pos_error_integral + (p - p_ref)
        self.a_e = -1.0*np.dot(self.Kp,p-p_ref) -1.0*np.dot(self.Kd,v-v_ref) -1.0*np.dot(self.Ki,self.pos_error_integral)  # PID control law

        # anti-windup integral control
        if((self.Traw >= self.max_thrust) or (self.Traw <= self.min_thrust)):
            Ki = np.diag([0.0, 0.0, 0.0])
            self.a_e2 = -1.0*np.dot(self.Kp,p-p_ref) -1.0*np.dot(self.Kd,v-v_ref) -1.0*np.dot(Ki,self.pos_error_integral)  # PID control law
        else:
            self.a_e2 = self.a_e            


        # Desired acceleration
        a_des = self.a_e2 + a_ref + params.g*params.e3
        #a_des2 = self.a_e + a_ref + params.g*params.e3

        wzb = np.dot(Rbw, params.e3)          # body z-axis expressed in world frame
        self.Traw = self.mass*np.dot(wzb.T,a_des)[0][0]            
        #self.Traw2 = self.mass*np.dot(wzb.T,a_des2)[0][0]          # debugging only
        T = utils.saturate_scalar_minmax(self.Traw, self.max_thrust, self.min_thrust)    # Maximum possible thrust

        zb_des = a_des / np.linalg.norm(a_des)
        Rbw_des, psi_ref = self.desired_attitude(zb_des, Rbw_ref, Rbw)

        a_real = -params.g*params.e3 + (T/self.mass)*wzb
        Fd_dot = -1.0*np.dot(self.Kp,v-v_ref) -1.0*np.dot(self.Kd, a_real - a_ref) -1.0*np.dot(self.Ki, p - p_ref) + j_ref


        yc_des = np.array(df_flat.get_yc(psi_ref))   #transform to np.array 'cause comes as np.matrix
        xc_des = np.array(df_flat.get_xc(psi_ref))
        xb_des = np.cross(yc_des, zb_des, axis = 0)
        xb_des = xb_des/np.linalg.norm(xb_des)

        Rbw_des_dot = self.desired_attitude_dot(a_real, Fd_dot, xc_des, yc_des, xb_des, zb_des, yawdot_ref )
        w_des = utils.vex2(np.dot(Rbw_des.T, Rbw_des_dot))
        #w_des = np.zeros((3,1))  
        return T, Rbw_des, w_des

    def attitude_controller(self):
        """
        """
        return 0

    def desired_attitude(self, zb_des, Rbw_ref, Rbw):
        """
        """
        Rbw_ref_h = utils.to_homogeneous_transform(Rbw_ref)
        psi_ref, theta_ref, phi_ref = tf.transformations.euler_from_matrix(Rbw_ref_h, axes = 'rzyx')

        yc_des = np.array(df_flat.get_yc(psi_ref))
        xb_des = np.cross(yc_des, zb_des, axis = 0)
        xb_des = xb_des/np.linalg.norm(xb_des)
        yb_des = np.cross(zb_des, xb_des, axis = 0)

        Rbw_des1 = np.concatenate((xb_des, yb_des, zb_des), axis = 1)
        Rbw_des2 = np.concatenate((-xb_des, -yb_des, zb_des), axis = 1)
        
        angle1 = utils.rotation_distance(Rbw, Rbw_des1)
        angle2 = utils.rotation_distance(Rbw, Rbw_des2)
        
        if(angle1 > angle2):
            #print("Rbw_des2 selected")
            Rbw_des = Rbw_des2
            xb_des = -xb_des
            yb_des = -yb_des
        else:
            #print("Rbw_des1 selected")
            Rbw_des = Rbw_des1

        return Rbw_des, psi_ref

    def desired_attitude_dot(self, Fd, Fd_dot, xc_des, yc_des, xb_des, zb_des, yawdot ):
        """
        Here following: 
        Lee, T., Leok, M., & McClamroch, N. H. (2010). 
        Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3), 1-14. 
        Retrieved ONLY from http://arxiv.org/abs/1003.2005v3

        Using R_dot = [x_dot, y_dot, z_dot]
        """
        Fd_norm = np.linalg.norm(Fd)
        Fd_norm_dot = np.dot(Fd.T,Fd_dot)[0][0]/Fd_norm
        zb_des_dot = (Fd_norm*Fd_dot - Fd*Fd_norm_dot)/(Fd_norm**2)

        C = np.cross(yc_des, zb_des, axis=0)
        C_norm = np.linalg.norm(C)
        C_dot = -1.0*yawdot*np.cross(xc_des, zb_des, axis=0) + np.cross(yc_des, zb_des_dot, axis=0)
        C_norm_dot = np.dot(C.T, C_dot)[0][0]/C_norm

        xb_des_dot = (C_norm*C_dot - C*C_norm_dot)/(C_norm**2)

        yb_des_dot = np.cross(zb_des_dot, xb_des, axis= 0) + np.cross(zb_des, xb_des_dot, axis = 0)

        Rbw_des_dot = np.concatenate((xb_des_dot, yb_des_dot, zb_des_dot), axis=1)

        return Rbw_des_dot


def main():
    gc = Geometric_Controller()


if __name__ == '__main__':
    main()
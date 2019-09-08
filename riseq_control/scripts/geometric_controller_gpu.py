"""
author:  Juan Medrano
version: 0.1
brief: A position and orientation controller for a quadrotor computed on GPU. Based on:
        Lee, T., Leok, M., & McClamroch, N. H. (2010). 
        Control of Complex Maneuvers for a Quadrotor UAV using Geometric 
        Methods on SE(3), 1-14. Retrieved from http://arxiv.org/abs/1003.2005

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
import control_gains as gains
import riseq_common.torch_dyn_utils as utils
import riseq_common.torch_physical_parameters as params
import riseq_common.differential_flatness_torch as df_flat
import torch
torch.set_default_dtype(torch.float32)
torch.set_default_tensor_type(torch.FloatTensor)

class GPU_Geometric_Controller():

    def __init__(self, mass= 1.0, max_thrust = 1.0, min_thrust = 0.0, inertia = torch.eye(n=3)):
        """
        Initialize this controller's parameters

        Args:
            mass (float): mass of the rigid body in kilograms
            max_thrust (float): maximum value of collective thrust
            min_thrust (float): minimum value of collective thrust
            inertia (3x3 np.array): inertia tensor of the quadrotor
        """
        """
        r = rospy.Rate(1)
        self.ready = False
        while not self.ready:
            try:
                torch.tensor([1.0])
                self.ready = True
                print(" >> GPU Controller READY: {}".format(torch.__version__))
            except:
                print(" >> GPU Controller not ready")
            r.sleep()
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print("Found {} device".format(self.device)) 

        self.mass = mass #torch.tensor(mass).to(self.device)
        self.Traw = torch.tensor(0.0).to(self.device)
        self.Fd_dot = torch.tensor(0.0).to(self.device)
        self.pos_error_integral = torch.zeros(3).to(self.device)

        # PID gains for position error controller 
        """
        

        self.Kp = torch.tensor([[gains.Kpx2,0.0,0.0],
                            [0.0,gains.Kpy2,0.0],
                            [0.0,0.0,gains.Kpz2]]).to(self.device)
        self.Kd = torch.tensor([[gains.Kdx2,0.0,0.0],
                            [0.0,gains.Kdy2,0.0], 
                            [0.0,0.0,gains.Kdz2]]).to(self.device)
        self.Ki = torch.tensor([[gains.Kix2,0.0,0.0], 
                            [0.0,gains.Kiy2,0.0], 
                            [0.0,0.0,gains.Kiz2]]).to(self.device)
        
        self.Kp = torch.tensor([[8.0,0.0,0.0],
                            [0.0,8.0,0.0],
                            [0.0,0.0,8.0]]).to(self.device)
        self.Kd = torch.tensor([[1.5,0.0,0.0],
                            [0.0,1.5,0.0], 
                            [0.0,0.0,10.3]]).to(self.device)
        self.Ki = torch.tensor([[0.0,0.0,0.0], 
                            [0.0,0.0,0.0], 
                            [0.0,0.0,0.0]]).to(self.device)        
        """
        self.Kp_gains = torch.tensor([[gains.Kpx2], [gains.Kpy2], [gains.Kpz2]]).squeeze(1).to(self.device)
        self.Kd_gains = torch.tensor([[gains.Kdx2], [gains.Kdy2], [gains.Kdz2]]).squeeze(1).to(self.device)
        self.Ki_gains = torch.tensor([[gains.Kix2], [gains.Kiy2], [gains.Kiz2]]).squeeze(1).to(self.device)

        self.Kp = torch.diag(self.Kp_gains)
        self.Kd = torch.diag(self.Kd_gains)
        self.Ki = torch.diag(self.Ki_gains)

        self.max_thrust = max_thrust #torch.tensor(max_thrust).to(self.device)
        self.min_thrust = min_thrust #torch.tensor(min_thrust).to(self.device)

    def position_controller(self, data):
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
        data = data.to(self.device)
        p = data[0]
        v = data[1]
        Rbw = data[5:8]

        p_ref = data[8]
        v_ref = data[9]
        a_ref = data[10]
        j_ref = data[11]
        #s_ref = ref[4]
        Rbw_ref = data[13:16]
        yawdot_ref = data[16,1]

        # Calculate  PID control law for acceleration error: 
        self.pos_error_integral = self.pos_error_integral + torch.add(p, -1.0, p_ref)#.type(torch.cuda.FloatTensor)
        self.a_e = torch.zeros(3).to(self.device)
        #print(" >> \n{},\n{},\n{}".format(self.a_e.dtype, self.Kp_gains.dtype,torch.add(p,-1.0,p_ref).dtype))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Kp_gains, torch.add(p,-1.0,p_ref))#.type(torch.cuda.FloatTensor))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Kd_gains, torch.add(v,-1.0,v_ref))#.type(torch.cuda.FloatTensor))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Ki_gains, self.pos_error_integral)

        # Desired acceleration and thrust
        #print(" >> \n{},\n{},\n{}".format(self.a_e, a_ref,torch.mul(params.e3,params.g)))
        a_des = self.a_e + a_ref + torch.mul(params.e3,params.g)#.type(torch.cuda.FloatTensor)
        #a_des2 = self.a_e + a_ref + params.g*params.e3
        wzb = torch.mm(Rbw, params.e3.view(3,-1)).squeeze(1)          # body z-axis expressed in world frame
        #print(wzb,a_des)
        self.Traw = self.mass*torch.dot(wzb,a_des)           
        #self.Traw2 = self.mass*np.dot(wzb.T,a_des2)[0][0]          # debugging only
        T = torch.clamp(self.Traw, self.min_thrust, self.max_thrust)    # Maximum possible thrust
        
        # Desired attitude
        zb_des = a_des / torch.norm(a_des)
        Rbw_des, psi_ref = self.desired_attitude(zb_des, Rbw_ref, Rbw)

        # Desired body rate
        a_real = torch.mul(params.e3, -params.g) + (T/self.mass)*wzb
        Fd_dot = torch.zeros(3).to(self.device)
        #print(" >> \n{},\n{},\n{}".format(self.a_e.dtype, self.Kp_gains.dtype,torch.add(p,-1.0,p_ref).dtype))
        Fd_dot = torch.addcmul(Fd_dot, -1.0, self.Kp_gains, torch.add(v,-1.0,v_ref))#.type(torch.cuda.FloatTensor))
        Fd_dot = torch.addcmul(Fd_dot, -1.0, self.Kd_gains, torch.add(a_real,-1.0,a_ref))#.type(torch.cuda.FloatTensor))
        Fd_dot = torch.addcmul(Fd_dot, -1.0, self.Ki_gains, torch.add(p,-1.0,p_ref))#.type(torch.cuda.FloatTensor))        
        Fd_dot = torch.add(Fd_dot, 1.0, j_ref)


        #Fd_dot = -1.0*np.dot(self.Kp,v-v_ref) -1.0*np.dot(self.Kd, a_real - a_ref) -1.0*np.dot(self.Ki, p - p_ref) + j_ref

        yc_des = df_flat.get_yc(psi_ref)   #transform to np.array 'cause comes as np.matrix
        xc_des = df_flat.get_xc(psi_ref)
        xb_des = torch.cross(yc_des, zb_des)
        xb_des = xb_des/torch.norm(xb_des)

        Rbw_des_dot = self.desired_body_rate(a_real, Fd_dot, xc_des, yc_des, xb_des, zb_des, yawdot_ref )
        w_des = utils.vex2(torch.mm(torch.t(Rbw_des), Rbw_des_dot))

        return T.item(), Rbw_des.cpu().numpy(), w_des.cpu().numpy()

    def position_controller2(self, state, ref):
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

        p = torch.from_numpy(state[0]).squeeze(1).type(torch.FloatTensor).to(self.device)
        v = torch.from_numpy(state[1]).squeeze(1).type(torch.FloatTensor).to(self.device)
        Rbw = torch.from_numpy(state[5]).squeeze(1).type(torch.FloatTensor).to(self.device)

        p_ref = torch.from_numpy(ref[0]).squeeze(1).type(torch.FloatTensor).to(self.device)
        v_ref = torch.from_numpy(ref[1]).squeeze(1).type(torch.FloatTensor).to(self.device)
        a_ref = torch.from_numpy(ref[2]).squeeze(1).type(torch.FloatTensor).to(self.device)
        j_ref = torch.from_numpy(ref[3]).squeeze(1).type(torch.FloatTensor).to(self.device)
        #s_ref = ref[4]
        Rbw_ref = torch.from_numpy(ref[5]).type(torch.FloatTensor).to(self.device)
        yawdot_ref = torch.tensor([ref[7]]).type(torch.FloatTensor).to(self.device)

        # Calculate  PID control law for acceleration error: 
        self.pos_error_integral = self.pos_error_integral + torch.add(p, -1.0, p_ref)#.type(torch.cuda.FloatTensor)
        self.a_e = torch.zeros(3).to(self.device)
        #print(" >> \n{},\n{},\n{}".format(self.a_e.dtype, self.Kp_gains.dtype,torch.add(p,-1.0,p_ref).dtype))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Kp_gains, torch.add(p,-1.0,p_ref))#.type(torch.cuda.FloatTensor))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Kd_gains, torch.add(v,-1.0,v_ref))#.type(torch.cuda.FloatTensor))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Ki_gains, self.pos_error_integral)

        # Desired acceleration and thrust
        #print(" >> \n{},\n{},\n{}".format(self.a_e, a_ref,torch.mul(params.e3,params.g)))
        a_des = self.a_e + a_ref + torch.mul(params.e3,params.g)#.type(torch.cuda.FloatTensor)
        #a_des2 = self.a_e + a_ref + params.g*params.e3
        wzb = torch.mm(Rbw, params.e3.view(3,-1)).squeeze(1)          # body z-axis expressed in world frame
        #print(wzb,a_des)
        self.Traw = self.mass*torch.dot(wzb,a_des)           
        #self.Traw2 = self.mass*np.dot(wzb.T,a_des2)[0][0]          # debugging only
        T = torch.clamp(self.Traw, self.min_thrust, self.max_thrust)    # Maximum possible thrust
        
        # Desired attitude
        zb_des = a_des / torch.norm(a_des)
        Rbw_des, psi_ref = self.desired_attitude(zb_des, Rbw_ref, Rbw)

        # Desired body rate
        a_real = torch.mul(params.e3, -params.g) + (T/self.mass)*wzb
        Fd_dot = torch.zeros(3).to(self.device)
        #print(" >> \n{},\n{},\n{}".format(self.a_e.dtype, self.Kp_gains.dtype,torch.add(p,-1.0,p_ref).dtype))
        Fd_dot = torch.addcmul(Fd_dot, -1.0, self.Kp_gains, torch.add(v,-1.0,v_ref))#.type(torch.cuda.FloatTensor))
        Fd_dot = torch.addcmul(Fd_dot, -1.0, self.Kd_gains, torch.add(a_real,-1.0,a_ref))#.type(torch.cuda.FloatTensor))
        Fd_dot = torch.addcmul(Fd_dot, -1.0, self.Ki_gains, torch.add(p,-1.0,p_ref))#.type(torch.cuda.FloatTensor))        
        Fd_dot = torch.add(Fd_dot, 1.0, j_ref)


        #Fd_dot = -1.0*np.dot(self.Kp,v-v_ref) -1.0*np.dot(self.Kd, a_real - a_ref) -1.0*np.dot(self.Ki, p - p_ref) + j_ref

        yc_des = df_flat.get_yc(psi_ref)   #transform to np.array 'cause comes as np.matrix
        xc_des = df_flat.get_xc(psi_ref)
        xb_des = torch.cross(yc_des, zb_des)
        xb_des = xb_des/torch.norm(xb_des)

        Rbw_des_dot = self.desired_body_rate(a_real, Fd_dot, xc_des, yc_des, xb_des, zb_des, yawdot_ref )
        w_des = utils.vex2(torch.mm(torch.t(Rbw_des), Rbw_des_dot))

        return T.item(), Rbw_des.cpu().numpy(), w_des.cpu().numpy()

    def desired_attitude(self, zb_des, Rbw_ref, Rbw):
        """
        Calculate a desired attitude based on a desired direction of body z-axis vector,
        a reference orientation and the current orientation 

        Args:
            zb_des (3x1 np.array): desired body zb axis
            Rbw_ref (3x3 np.array): reference 3x3 orientation as rotation matrix
            Rbw (3x1 np.array): current 3x3 orientation matrix

        Returns:
            Desired attitude as a 3x3 rotation matrix (np.array) and reference yaw angle      
        """
        phi_ref, theta_ref, psi_ref = utils.euler_from_matrix(Rbw_ref)
        yc_des = df_flat.get_yc(psi_ref)
        xb_des = torch.cross(yc_des, zb_des)
        xb_des = xb_des/torch.norm(xb_des)
        yb_des = torch.cross(zb_des, xb_des)

        Rbw_des1 = torch.cat((xb_des.view(3,-1), yb_des.view(3,-1), zb_des.view(3,-1)), dim = 1)
        Rbw_des2 = torch.cat(((-xb_des).view(3,-1), (-yb_des).view(3,-1), zb_des.view(3,-1)), dim = 1)
        
        angle1 = utils.rotation_matrix_distance(Rbw, Rbw_des1)
        angle2 = utils.rotation_matrix_distance(Rbw, Rbw_des2)
        
        # Find rotation with the smallest angle to the
        # current orientation
        if(torch.ge(angle1, angle2).item()):
            Rbw_des = Rbw_des2
            xb_des = -xb_des
            yb_des = -yb_des
        else:
            Rbw_des = Rbw_des1

        return Rbw_des, psi_ref

    def desired_body_rate(self, Fd, Fd_dot, xc_des, yc_des, xb_des, zb_des, yawdot ):
        """
        Compute desired attitude rate as a rotation matrix, in other words, the desired rotation
        matrix rate, which is equal to the rate of each of the 3 column vectors:
                              R_dot = [x_dot, y_dot, z_dot]

        Args:
            Fd (3x1 np.array): mass normalized thrust vector 
            Fd (3x1 np.array): mass normalized thrust vector derivative
            xc_des (3x1 np.array): x-axis of the x-y plane projection of the desired orientation matrix
            yc_des (3x1 np.array): y-axis of the x-y plane projection of the desired orientation matrix
            xb_des (3x1 np.array): x axis of the desired orientation matrix
            zb_des (3x1 np.array): z axis of the desired orientation matrix
            yawdot (float): desired yaw angle in radians

        Reference: 
        Lee, T., Leok, M., & McClamroch, N. H. (2010). 
        Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3), 1-14. 
        Retrieved ONLY from http://arxiv.org/abs/1003.2005v3

        Using 
        """
        Fd_norm = torch.norm(Fd)
        Fd_norm_dot = torch.dot(Fd.T,Fd_dot)/Fd_norm
        zb_des_dot = (torch.mul(Fd_dot, Fd_norm) - torch.mul(Fd,Fd_norm_dot))/(Fd_norm**2)

        C = torch.cross(yc_des, zb_des)
        C_norm = torch.norm(C)
        C_dot = -1.0*yawdot*torch.cross(xc_des, zb_des) + torch.cross(yc_des, zb_des_dot)
        C_norm_dot = torch.dot(C, C_dot)/C_norm

        xb_des_dot = (torch.mul(C_dot,C_norm) - torch.mul(C,C_norm_dot))/(C_norm**2)

        yb_des_dot = torch.cross(zb_des_dot, xb_des) + torch.cross(zb_des, xb_des_dot)

        Rbw_des_dot = torch.cat((xb_des_dot.view(3,-1), yb_des_dot.view(3,-1), zb_des_dot.view(3,-1)), dim=1)

        return Rbw_des_dot


def main():
    gc = GPU_Geometric_Controller()

if __name__ == '__main__':
    main()
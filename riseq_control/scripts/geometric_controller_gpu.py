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
import riseq_common.dyn_utils as utils
import riseq_common.torch_physical_parameters as params
import riseq_tests.df_flat as df_flat
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

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu') 

        self.mass = mass #torch.tensor(mass).to(self.device)
        self.Traw = torch.tensor(0.0).to(self.device)
        self.Fd_dot = torch.tensor(0.0).to(self.device)
        self.pos_error_integral = torch.zeros(3,1).to(self.device)

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
        self.Kp_gains = torch.tensor([[gains.Kpx2], [gains.Kpy2], [gains.Kpz2]]).to(self.device)
        self.Kd_gains = torch.tensor([[gains.Kdx2], [gains.Kdy2], [gains.Kdz2]]).to(self.device)
        self.Ki_gains = torch.tensor([[gains.Kix2], [gains.Kiy2], [gains.Kiz2]]).to(self.device)

        self.Kp = torch.diag(self.Kp_gains)
        self.Kd = torch.diag(self.Kd_gains)
        self.Ki = torch.diag(self.Ki_gains)

        self.max_thrust = max_thrust #torch.tensor(max_thrust).to(self.device)
        self.min_thrust = min_thrust #torch.tensor(min_thrust).to(self.device)


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

        p = torch.from_numpy(state[0]).type(torch.FloatTensor).to(self.device)
        v = torch.from_numpy(state[1]).type(torch.FloatTensor).to(self.device)
        Rbw = torch.from_numpy(state[5]).type(torch.FloatTensor).to(self.device)

        p_ref = torch.from_numpy(ref[0]).type(torch.FloatTensor).to(self.device)
        v_ref = torch.from_numpy(ref[1]).type(torch.FloatTensor).to(self.device)
        a_ref = torch.from_numpy(ref[2]).type(torch.FloatTensor).to(self.device)
        j_ref = torch.from_numpy(ref[3]).type(torch.FloatTensor).to(self.device)
        #s_ref = ref[4]
        Rbw_ref = torch.from_numpy(ref[5]).type(torch.FloatTensor).to(self.device)
        yawdot_ref = torch.tensor([ref[7]]).type(torch.FloatTensor).to(self.device)

        # Calculate  PID control law for acceleration error: 
        #self.pos_error_integral = self.pos_error_integral + (p - p_ref)
        #self.a_e = -1.0*np.dot(self.Kp,p-p_ref) -1.0*np.dot(self.Kd,v-v_ref) -1.0*np.dot(self.Ki,self.pos_error_integral)  # PID control law
        self.pos_error_integral = self.pos_error_integral + torch.add(p, -1.0, p_ref)#.type(torch.cuda.FloatTensor)
        self.a_e = torch.zeros(3,1).to(self.device)
        #print(" >> \n{},\n{},\n{}".format(self.a_e.dtype, self.Kp_gains.dtype,torch.add(p,-1.0,p_ref).dtype))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Kp_gains, torch.add(p,-1.0,p_ref))#.type(torch.cuda.FloatTensor))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Kd_gains, torch.add(v,-1.0,v_ref))#.type(torch.cuda.FloatTensor))
        self.a_e = torch.addcmul(self.a_e, -1.0, self.Ki_gains, self.pos_error_integral)

        # Desired acceleration and thrust
        #print(" >> \n{},\n{},\n{}".format(self.a_e.dtype, a_ref.dtype,torch.mul(params.e3,params.g).dtype))
        a_des = self.a_e + a_ref + torch.mul(params.e3,params.g)#.type(torch.cuda.FloatTensor)
        #a_des2 = self.a_e + a_ref + params.g*params.e3
        wzb = torch.mm(Rbw, params.e3)          # body z-axis expressed in world frame
        self.Traw = self.mass*torch.dot(wzb.squeeze(1),a_des.squeeze(1))           
        #self.Traw2 = self.mass*np.dot(wzb.T,a_des2)[0][0]          # debugging only
        T = torch.clamp(self.Traw, self.min_thrust, self.max_thrust)    # Maximum possible thrust
        
        return T.item()


def main():
    gc = GPU_Geometric_Controller()

if __name__ == '__main__':
    main()
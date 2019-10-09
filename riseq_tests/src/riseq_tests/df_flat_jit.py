from __future__ import division
import torch
import torch.nn as nn
torch.set_default_dtype(torch.float64)


import numpy as np
import matplotlib.pyplot as plt

mass = 1.0
g = 9.8
sim_time = 0
class DroneFlatSystem(nn.Module):

    __constants__=['g', 'm']

    def __init__(self, g = 9.81, m = 1.0, I = torch.eye(3)):
        super(DroneFlatSystem, self).__init__()
        self.g = nn.Parameter(torch.tensor([0.,0.,g]))
        self.m = nn.Parameter(torch.tensor([m]))
        self.I = nn.Parameter(I)
        self.invI = nn.Parameter(torch.inverse(self.I))

    #@torch.jit.script_method
    def forward(self, flat_input):
        # type: (Tensor) -> Tensor 
        
        # extract all quantities from given flat_input
        flat_pos = flat_input[0]  # 3-vector
        flat_vel = flat_input[1]  # 3-vector
        flat_acc = flat_input[2]  # 3-vector
        flat_jerk = flat_input[3]  # 3-vector
        flat_snap = flat_input[4]  # 3-vector
        flat_yaw = float(flat_input[5][0].item())  # scalar
        flat_yaw_dot = float(flat_input[5][1].item())  # scalar
        flat_yaw_ddot = float(flat_input[5][2].item())  # scalar

        t_vec = self.get_t_vector(flat_acc)
        u_1 = self.get_u1(t_vec)
        z_b = self.get_zb(t_vec)
        y_c = self.get_yc(flat_yaw)
        x_b = self.get_xb(y_c, z_b)
        y_b = self.get_yb(z_b, x_b)
        w_x = self.get_wx(y_b, flat_jerk, u_1)
        w_y = self.get_wy(x_b, flat_jerk, u_1)  
        x_c = self.get_xc(flat_yaw)
        w_z = self.get_wz(flat_yaw_dot, x_c, x_b, w_y, y_c, z_b)  
        u_1_dot = self.get_u1_dot(z_b, flat_jerk)
        w_y_dot = self.get_wy_dot(x_b, flat_snap, u_1_dot, w_y, u_1, w_x, w_z)
        w_x_dot = self.get_wx_dot(y_b, flat_snap, u_1_dot, w_x, u_1, w_y, w_z)  
        w_z_dot = self.get_wz_dot(flat_yaw_ddot, x_c, x_b, flat_yaw_dot, w_z, y_b, w_y, z_b, w_x, y_c, w_y_dot)
        w_dot_ = torch.cat((w_x_dot, w_y_dot, w_z_dot), dim = 0)
        w_ = torch.cat((w_x, w_y, w_z), dim = 0)

        # get vector fo torque inputs
        u_x = self.get_ux(w_dot_, w_)

        R_ = self.matrix_from_vectors(x_b, y_b, z_b)
        or_ = self.RotToRPY_ZYX(R_)
        u_a = self.get_ua(u_1, z_b)  
        u_b = self.get_ub(w_, u_x)  
        u_c = self.get_uc(w_, or_) 

        u_1 = torch.cat((torch.tensor([0.0]), torch.tensor([0.0]), u_1), dim = 0) # pack u_1 in a tensor
        ref_state = torch.cat((flat_pos.unsqueeze(0), flat_vel.unsqueeze(0), or_.unsqueeze(0), 
                               w_.unsqueeze(0), w_dot_.unsqueeze(0), u_a.unsqueeze(0), u_b.unsqueeze(0), 
                               u_c.unsqueeze(0), u_1.unsqueeze(0), u_x.unsqueeze(0), 
                               R_, flat_acc.unsqueeze(0), flat_jerk.unsqueeze(0), 
                               flat_snap.unsqueeze(0), flat_input[5].unsqueeze(0)),dim=0)

        return ref_state

    def get_u1(self, t):
        # type: (Tensor) -> Tensor
        u1 = self.m * torch.norm(t)
        return u1

    def get_zb(self, t):
        # type: (Tensor) -> Tensor
        zb = t / (torch.norm(t))
        return zb   

    def get_t_vector(self, acc):
        # type: (Tensor) -> Tensor
        return acc + self.g
    
    def get_yc(self, yaw):
        # type: (float) -> Tensor
        y_c = torch.tensor([-1.0 * torch.sin(yaw), torch.cos(yaw), 0.0])
        return y_c

    def get_xc(self, yaw):
        # type: (float) -> Tensor
        x_c = torch.tensor([torch.cos(yaw), torch.sin(yaw), 0.0])
        return x_c    

    def get_xb(self, y_c, z_b):
        # type: (Tensor, Tensor) -> Tensor
        a = torch.cross(y_c, z_b)
        return a / torch.norm(a)

    def get_yb(self, z_b, x_b):
        # type: (Tensor, Tensor) -> Tensor
        a = torch.cross(z_b, x_b)
        return a / torch.norm(a)

    def get_wx(self, y_b, j, u_1):
        # type: (Tensor, Tensor, Tensor) -> Tensor
        w_x = -1.0 * self.m * torch.dot(y_b, j) / u_1
        return w_x

    def get_wy(self, x_b, j, u_1):
        # type: (Tensor, Tensor, Tensor) -> Tensor
        w_y = self.m * torch.dot(x_b, j) / u_1
        return w_y

    def get_wz(self, psi_rate, x_c, x_b, w_y, y_c, z_b):
        # type: (float, Tensor, Tensor, Tensor, Tensor, Tensor) -> Tensor
        a = psi_rate * torch.dot(x_c, x_b)
        b = w_y * torch.dot(y_c, z_b)
        c = torch.norm(torch.cross(y_c, z_b))
        w_z = (a + b) / c
        return w_z

    def get_u1_dot(self, z_b, j):
        # type: (Tensor, Tensor) -> Tensor
        u1_dot = self.m * torch.dot(z_b, j)
        return u1_dot

    def get_wy_dot(self, x_b, s, u_1_dot, w_y, u_1, w_x, w_z):
        # type: (Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor) -> Tensor
        """
            Will use wy_dot = (a + b + c)/d
        """
        a = torch.dot(x_b, s)
        b = -2.0 * u_1_dot * w_y / self.m
        c = -1.0 * u_1 * w_x * w_z / self.m
        d = u_1 / self.m
        w_y_dot = (a + b + c) / d
        return w_y_dot

    def get_wx_dot(self, y_b, s, u_1_dot, w_x, u_1, w_y, w_z):
        # type: (Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor) -> Tensor
        """
            Will use wx_dot = (a + b + c)/d
        """
        a = -1.0 * torch.dot(y_b, s)
        b = -2.0 * u_1_dot * w_x / self.m
        c = u_1 * w_y * w_z / self.m
        d = u_1 / self.m
        w_x_dot = (a + b + c) / d
        return w_x_dot

    def get_wz_dot(self, psi_acc, x_c, x_b, psi_rate, w_z, y_b, w_y, z_b, w_x, y_c, w_y_dot):
        # type: (float, Tensor, Tensor, float, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor, Tensor) -> Tensor
        """
            Will compute as w_z_dot = (a+b+c+d+e+f)/g
        """
        a = psi_acc * torch.dot(x_c, x_b)
        b = 2.0 * psi_rate * w_z * torch.dot(x_c, y_b)
        c = -2.0 * psi_rate * w_y * torch.dot(x_c, z_b)
        d = -1.0 * w_x * w_y * torch.dot(y_c, y_b)
        e = -1.0 * w_x * w_z * torch.dot(y_c, z_b)
        f = w_y_dot * torch.dot(y_c, z_b)
        g = torch.norm(torch.cross(y_c, z_b))
        w_z_dot = (a + b + c + d + e + f) / g
        return w_z_dot

    def get_ux(self, w_dot_, w_):
        # type: (Tensor, Tensor) -> Tensor
        u_x = torch.mm(self.I, w_dot_.view(-1,1)).squeeze() + torch.cross(w_, torch.mm(self.I , w_.view(-1,1)).squeeze())
        return u_x

    def matrix_from_vectors(self, x, y, z):
        # type: (Tensor, Tensor, Tensor) -> Tensor
        R = torch.cat((x.view(-1,1),y.view(-1,1),z.view(-1,1)), dim=1)
        return R

    def RotToRPY_ZYX(self, R):
        # type: (Tensor) -> Tensor
        """
            Euler angle convention is ZYX, which means first apply
            rotaion of psi-degrees around Z axis, then rotation of
            theta-degrees around new Y axis, and then rotation of
            phi-degrees around new X axis.
            ** The rotation R received should be from body to world frame. **
        """
        theta = torch.asin(-1.0 * R[2, 0])
        phi = torch.atan2(R[2, 1] / torch.cos(theta), R[2, 2] / torch.cos(theta))
        psi = torch.atan2(R[1, 0] / torch.cos(theta), R[0, 0] / torch.cos(theta))
        return torch.tensor([float(phi.item()), float(theta.item()), float(psi.item())])

    def get_ua(self, u_1, z_b):
        # type: (Tensor, Tensor) -> Tensor
        """
            ua = -g*z_w +u1*z_b/m
        """
        u_a = -1.0*self.g + (u_1 * z_b) / self.m
        return u_a

    def get_ub(self, w_, M):
        # type: (Tensor, Tensor) -> Tensor
        u_b = torch.mm(self.invI, (-1.0 * torch.cross(w_, torch.mm(self.I, w_.view(-1,1)).squeeze()) + M).view(-1,1)).squeeze()
        return u_b

    def get_uc(self, w_, ori):
        # type: (Tensor, Tensor) -> Tensor
        """
        """
        phi_ = float(ori[0].item())
        theta_ = float(ori[1].item())
        psi_ = float(ori[2].item())

        peta = torch.tensor([
            [1.0, torch.sin(phi_) * torch.tan(theta_), torch.cos(phi_) * torch.tan(theta_)],
            [0.0, torch.cos(phi_), -1.0 * torch.sin(phi_)],
            [0.0, torch.sin(phi_) / torch.cos(theta_), torch.cos(phi_) / torch.cos(theta_)]])
        u_c = torch.mm(peta , w_.view(-1,1)).squeeze()

        return u_c

def gen_helix_trajectory2(t):
    """
        This function returns the trajectory: position, velocity,
        acceleration, jerk and snap an object going through a 3D helix 
        should have.
    """
    a = 0.5
    b = 0.5
    c = 0.5

    wx = 0.9
    wy = 1.1

    x_0 = 1.0
    y_0 = 1.0
    z_0 = 0.0

    # positions in helix
    x = a*np.cos(wx*t) + x_0
    y = b*np.sin(wy*t) + y_0
    z = c*t
    #psi = 0.0*np.ones_like(t)
    #tangent_vector = map(lambda a,b,c: np.matrix([[a],[b],[0]]),-a*wx*np.sin(wx*t),b*wy*np.cos(wy*t),c)
    psi = np.sin(t)
    #psi = np.arccos( )

    # velocities in helix
    v_x = -a*wx*np.sin(wx*t)
    v_y = b*wy*np.cos(wy*t)
    v_z = c*np.ones_like(t)
    psi_rate = np.cos(t)#0.0*np.ones_like(t)

    # accelerations in helix
    a_x = -(wx**2)*(x - x_0)
    a_y = -(wy**2)*(y - y_0)
    a_z = 0.0*np.ones_like(t)
    psi_dd = -1.0*np.sin(t)#0.0*np.ones_like(t)

    # jerks in helix
    j_x = -(wx**2)*(v_x)
    j_y = -(wy**2)*(v_y)
    j_z = 0.0*np.ones_like(t)
    psi_ddd = -1.0*np.cos(t)#0.0*np.ones_like(t)

    # snap in helix
    s_x = -(wx**2)*(a_x)
    s_y = -(wy**2)*(a_y)
    s_z = 0.0*np.ones_like(t)
    psi_dddd = np.sin(t) #0.0*np.ones_like(t)

    # pack everything
    pos = np.array([x,y,z])
    vel = np.array([v_x,v_y,v_z])
    acc = np.array([a_x,a_y,a_z])
    jerk = np.array([j_x,j_y,j_z])
    snap = np.array([s_x,s_y,s_z])
    psi_ref = np.array([psi,psi_rate,psi_dd])

    return np.stack((pos,vel,acc,jerk,snap,psi_ref), axis = 0)
    #return [pos,vel,acc,jerk,snap, psi_ref]#, psi_ddd, psi_dddd]

def draw_output(states, figs):


    # Euler angles
    phi1 = map(lambda a: a[2][0][0]*180.0/np.pi, states)
    theta1 = map(lambda a: a[2][1][0]*180.0/np.pi, states)
    psi1 = map(lambda a: a[2][2][0]*180.0/np.pi, states)

    figs[0] = add_plots(figs[0],sim_time,[phi1,theta1,psi1],["-","-","-"],["r","g","b"],["phi","theta","psi"],"Angular position of quadrotor",'t {s}','phi, theta, psi {degree}')
    figs[0].legend(loc='lower right', shadow=True, fontsize='small')    

    # Angular velocities
    wx1 = map(lambda a: a[3][0][0], states)
    wy1 = map(lambda a: a[3][1][0], states)
    wz1 = map(lambda a: a[3][2][0], states)
    #print(wx1)
    figs[1] = add_plots(figs[1],sim_time,[wx1,wy1,wz1],["-","-","-"],["r","g","b"],["wx","wy","wz"],"Angular velocities of quadrotor",'t {s}','wx, wy, wz {degree/s}')
    figs[1].legend(loc='lower right', shadow=True, fontsize='small')

    # Angular accelerations
    wx1_dot = map(lambda a: a[4][0][0], states)
    wy1_dot = map(lambda a: a[4][1][0], states)
    wz1_dot = map(lambda a: a[4][2][0], states)
    #print(wx1)
    figs[2] = add_plots(figs[2],sim_time,[wx1_dot,wy1_dot,wz1_dot],["-","-","-"],["r","g","b"],["wx_dot","wy_dot","wz_dot"],"Angular acceleration of quadrotor",'t {s}','wx, wy, wz {degree/s2}')
    figs[2].legend(loc='lower right', shadow=True, fontsize='small')    

    # control torque
    ux1 = map(lambda a: a[9][0][0], states)
    uy1 = map(lambda a: a[9][1][0], states)
    uz1 = map(lambda a: a[9][2][0], states)
    #print(wx1)
    figs[3] = add_plots(figs[3],sim_time,[ux1,uy1,uz1],["-","-","-"],["r","g","b"],["ux","uy","uz"],"Control Torque",'t {s}','ux, uy, uz {Nm}')
    figs[3].legend(loc='lower right', shadow=True, fontsize='small')    

    # control thrust
    F_t = map(lambda a: a[8], states)
    #F_t_dot = map(lambda a: a[-1], states)
    #F_t_ddot = map(lambda a: a[-2], states)
    weight = mass*g*np.ones_like(F_t)
    figs[4] = add_plots(figs[4],sim_time,[F_t, weight],["-","--"],["r","k"],["F","m*g"],"Rotor Thrust -F-  over time",'t {s}','F {N}')
    #figs[4] = add_plots(figs[4],sim_time,[F_t, weight, F_t_dot, F_t_ddot],["-","--","-","-"],["r","k", "g", "b"],["F","m*g", "F'","F''"],"Rotor Thrust -F- and derivatives over time",'t {s}','F {N}')
    figs[4].legend(loc='lower right', shadow=True, fontsize='small') 

    # Euler rates
    phi1_dot = map(lambda a: a[7][0][0]*180.0/np.pi, states)
    theta1_dot = map(lambda a: a[7][1][0]*180.0/np.pi, states)
    psi1_dot = map(lambda a: a[7][2][0]*180.0/np.pi, states)

    figs[5] = add_plots(figs[5],sim_time,[phi1_dot,theta1_dot,psi1_dot],["-","-","-"],["r","g","b"],["phi_dot","theta_dot","psi_dot"],"Euler Rates",'t {s}','phi, theta, psi {degree/s}')
    figs[5].legend(loc='lower right', shadow=True, fontsize='small')  

def add_plots(ax,x,datas,lines,cols,labs,tit,xlab,ylab):

    for (data, line, colr, labl) in zip(datas, lines, cols, labs):
        ax.plot(x,data, linestyle = line, color = colr, label = labl)
    ax.set_title(tit)
    ax.set_xlabel(xlab)
    ax.set_ylabel(ylab)
    return ax

def to_list(s): 

    states = []
    for i in [0,1,2,3,4,5,6,7,8,9,10,13,14,15,16]:
        if i == 10:
            R = np.array([s[10].cpu().detach().numpy(),
                          s[11].cpu().detach().numpy(),
                          s[12].cpu().detach().numpy()])
            states.append(R)
        elif i == 8:
            T = s[8].cpu().detach().numpy()[2]
            states.append(T)
        else:
            states.append(s[i].view(-1,1).cpu().detach().numpy())

    return states

def main():
    global sim_time
    jit_difflat = torch.jit.script(DroneFlatSystem())

    tmax = 10
    step = 0.01
    sim_time = np.arange(0, tmax, step)

    flat_out_traj = []
    ref_states1 = []

    for t in sim_time:
        flat_out_state = gen_helix_trajectory2(t)
        flat_out_traj.append(flat_out_state)
        #flat_out_state = np.array(flat_out_state)
        #print(flat_out_state)

        # Generate flat state references
        flat_out_state = torch.from_numpy(flat_out_state)
        states = jit_difflat(flat_out_state)
        states = to_list(states)
        ref_states1.append(states)


    if(True):
        fig0 = plt.figure(figsize=(20,10))
        fig0.tight_layout()
        fig0ax0 = fig0.add_subplot(3,2,1)
        fig0ax1 = fig0.add_subplot(3,2,2)
        fig0ax2 = fig0.add_subplot(3,2,3)
        fig0ax3 = fig0.add_subplot(3,2,4)
        fig0ax4 = fig0.add_subplot(3,2,5)
        fig0ax5 = fig0.add_subplot(3,2,6) 
        fig0.suptitle("Differential Flatness")  
        figlist0 = [fig0ax0, fig0ax1, fig0ax2, fig0ax3, fig0ax4, fig0ax5]
        draw_output(ref_states1, figlist0)

    plt.show()

if __name__ == '__main__':
    main()
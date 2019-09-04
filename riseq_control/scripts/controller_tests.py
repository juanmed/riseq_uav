import numpy as np
import matplotlib.pyplot as plt
import torch

from geometric_controller import Geometric_Controller as gc
from geometric_controller_gpu import GPU_Geometric_Controller as ggc

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
    pos = np.array([[x],[y],[z]])
    vel = np.array([[v_x],[v_y],[v_z]])
    acc = np.array([[a_x],[a_y],[a_z]])
    jerk = np.array([[j_x],[j_y],[j_z]])
    snap = np.array([[s_x],[s_y],[s_z]])

    return [pos,vel,acc,jerk,snap, psi, psi_rate, psi_dd, psi_ddd, psi_dddd]

def Rx(roll):
    Rx = np.array([[1.0,0.0,0.0],
                   [0.0,np.cos(roll), np.sin(roll)],
                   [0.0,-np.sin(roll), np.cos(roll)]])
    return Rx

def Ry(pitch):
    Ry = np.array([[np.cos(pitch),0.0,-np.sin(pitch)],
                   [0.0,1.0,0.0],
                   [np.sin(pitch),0.0,np.cos(pitch)]])    
    return Ry

def Rz(yaw):
    Rz = np.array([[np.cos(yaw), np.sin(yaw),0.0],
                   [-np.sin(yaw), np.cos(yaw),0.0],
                   [0.0,0.0,1.0]])
    return Rz

def main():

    #warm up
    #device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    #q = np.random.rand(1000,1000)
    #t = np.random.rand(1000,1000)
    #np.dot(q,t)
    #q = torch.from_numpy(q).to(device)
    #t = torch.from_numpy(t).to(device)
    #torch.mm(q,t)

    tmax = 10
    step = 0.01
    sim_time = np.arange(0, tmax, step)

    #c = gc(max_thrust = 20.)
    c = ggc(max_thrust = 20.)

    Ts = []
    for t in sim_time:
    	traj = gen_helix_trajectory2(t)

        # extract ref trajectory
        pr = traj[0]
        vr = traj[1]
        ar = traj[2]
        jr = traj[3]
        sr = traj[4]
        yr = traj[5]
        ydr = traj[6]
        yddr = traj[7]
        Rr = np.dot(Rx(0.0),np.dot(Ry(0.0),Rz(yr)))
        ref_state = [pr, vr, ar, jr, sr, Rr, yr, ydr, yddr, 0.0]

        # create state
        s = 0.95
        R = np.dot(Rx(0.0),np.dot(Ry(0.0),Rz(yr*s))) 
        state = [pr*s, vr*s, ar*s, jr*s, sr*s, R, yr*s, ydr*s, yddr*s, 0.0]

        # compute control inputs
        T, Rd, wd = c.position_controller(state, ref_state)
        #Ts.append(T)
        #print(Ts)

    #fig = plt.figure()
    #ax = fig.add_subplot(1,1,1)
    #ax.plot(Ts)
    #plt.show()

if __name__ == '__main__':
	main()
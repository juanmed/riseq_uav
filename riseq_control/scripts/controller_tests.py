import numpy as np
import matplotlib.pyplot as plt

from geometric_controller import Geometric_Controller as gc
from geometric_controller_torch import GPU_Geometric_Controller as ggc

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

def main():

    tmax = 10
    step = 0.01
    sim_time = np.arange(0, tmax, step)

    c = gc()
    ct = ggc()

    for t in sim_time:
    	traj = gen_helix_trajectory2(t)
    	


if __name__ == '__main__':
	main()
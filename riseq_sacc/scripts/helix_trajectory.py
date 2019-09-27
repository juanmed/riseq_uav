
import rospy
import numpy as np
from SISO2 import SISO2_Controller as controller


class Helix_Trajectory_Control():

    def __init__(self, vrate = 0.1, radius = 1.0, center = (0,0,0), init = (0,0,0), t_init = 0.0, w = 1.0):

        self.vrate = vrate
        self.radius = radius
        self.w = w
        self.t_init = t_init
        #print("Helix Init Time: {}".format(t_init))

        self.cx = center[0]
        self.cy = center[1]

        self.x0 = init[0]
        self.y0 = init[1]
        self.z0 = init[2]

        self.xc = controller(Kp = 10.5, Kv = 1.0, Ki = 0.0)
        self.yc = controller(Kp = 8.0, Kv = 1.5, Ki = 0.0, u_sat = 5.0)

        self.phase = 0.0

    def compute_command(self, state, t):

        # compute reference
        xr, yr = self.get_reference_state(t)

        ux = self.xc.compute_input(state[0], xr)
        uy = self.yc.compute_input(state[1], yr)
        uz = self.z0 + self.vrate*(t-self.t_init)
        #print("Time Differences: ",t,self.t_init,t-self.t_init, rospy.Time.now().to_sec())
        return ux, uy, uz, (xr, yr)


    def get_reference_state(self, t):
        """
        """
        x = self.radius*np.cos(self.w*(t-self.t_init)+self.phase) + self.cx #- self.x0
        y = self.radius*np.sin(self.w*(t-self.t_init)+self.phase) + self.cy #- self.y0

        vx = -self.radius*self.w*np.sin(self.w*(t-self.t_init)+self.phase)
        vy = self.radius*self.w*np.cos(self.w*(t-self.t_init)+self.phase)

        ref1 = np.array([[x],[vx],[0.0]])
        ref2 = np.array([[y],[vy],[0.0]])
        return ref1, ref2

    def set_helix_center(self, c):
        self.cx = c[0]
        self.cy = c[1]
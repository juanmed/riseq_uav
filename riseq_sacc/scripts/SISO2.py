import numpy as np

class SISO2_Controller():

    def __init__(self, Kp = 5.0, Kv = 10.0, Ki = 0.0, u_sat = 1.0):
        
        self.Kp = Kp
        self.Kv = Kv
        self.Ki = Ki
        self.K = np.diag([Kp,Kv,Ki])
        self.x_error_integral  = 0.0
        self.input_saturation = u_sat


    def compute_input(self,state, desired):
        """
        """
        self.x_error_integral = self.x_error_integral + (state[0][0]-desired[0][0])
        if(np.abs(self.x_error_integral) > self.input_saturation):
            # saturate but with same sign
            self.x_error_integral = self.input_saturation*(self.x_error_integral/np.abs(self.x_error_integral))
        state[2][0] = self.x_error_integral
        u = -np.dot(self.K,state-desired).sum()
        return u 





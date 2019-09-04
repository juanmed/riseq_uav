import numpy as np

class SISO2_Controller():

    def __init__(self, Kp = 5.0, Kv = 10.0, Ki = 0.0):
        
        self.Kp = Kp
        self.Kv = Kv
        self.Ki = Ki
        self.K = np.diag([Kp,Kv,Ki])


    def compute_input(self,state, desired):
        """
        """
        u = -np.dot(self.K,state-desired).sum()
        return u 





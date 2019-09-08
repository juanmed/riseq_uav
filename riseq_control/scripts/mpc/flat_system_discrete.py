import control as ctl
import numpy as np


A = np.array([[0.0, 1.0, 0.0, 0.0],
			  [0.0, 0.0, 1.0, 0.0],
			  [0.0, 0.0, 0.0, 1.0],
			  [0.0, 0.0, 0.0, 0.0]])

B = np.array([[0.0],[0.0],[0.0],[1.0]])
C = np.array([1.0, 1.0, 1.0, 1.0])
D = np.array([0.0])

dt= 0.01

sys = ctl.StateSpace(A,B,C,D)
sysd = sys.sample(dt)
print(sysd)


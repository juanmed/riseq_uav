import torch as T
T.set_default_dtype(T.float32)
device = T.device('cuda' if T.cuda.is_available() else 'cpu')

g=9.8 # gravity, m/s2

# Vectors e1, e2, e3 generate R3
e1 = T.tensor([1.,0.,0.]).to(device)       
e2 = T.tensor([0.,1.,0.]).to(device)
e3 = T.tensor([0.,0.,1.]).to(device)
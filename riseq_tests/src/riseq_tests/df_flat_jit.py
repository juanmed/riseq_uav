import torch
 


@torch.jit.script
def get_u1(t):
	u1 = m * torch.norm(t)
	return u1

@torch.jit.script
def get_zb(t):
	zb = t / (torch.norm(t))
	return zb
	
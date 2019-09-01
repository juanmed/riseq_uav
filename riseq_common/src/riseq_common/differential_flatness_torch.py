import torch

def get_yc(sigma4):
    device = sigma4.device
    y_c = torch.tensor([-1.0 * torch.sin(sigma4), torch.cos(sigma4), 0.0]).to(device)
    return y_c

def get_xc(sigma4):
    device = sigma4.device
    x_c = torch.tensor([torch.cos(sigma4), torch.sin(sigma4), 0.0]).to(device)
    return x_c  
import tf.transformations as tt
import numpy as np
import torch
import riseq_common.torch_dyn_utils as utils



def main():

	device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

	roll = np.pi/9.0
	pitch = np.pi/5.0
	yaw = np.pi/2.0

	#R = tt.euler_matrix(roll, pitch, yaw, axes = 'rzyx')
	zaxis = np.array([[np.cos(yaw), np.sin(yaw),0.0],
				  [-np.sin(yaw), np.cos(yaw),0.0],
				  [0.0,0.0,1.0]])
	yaxis = np.array([[np.cos(pitch),0.0,-np.sin(pitch)],
				  [0.0,1.0,0.0],
				  [np.sin(pitch),0.0,np.cos(pitch)]])
	xaxis = np.array([[1.0,0.0,0.0],
					  [0.0,np.cos(roll), np.sin(roll)],
					  [0.0,-np.sin(roll), np.cos(roll)]])
	R = np.dot(xaxis,np.dot(yaxis,zaxis))
	R = torch.from_numpy(R.T).type(torch.FloatTensor).to(device)
	r,p,y = utils.euler_from_matrix(R)


	base = [roll,pitch, yaw] 
	res = [r.item(),p.item(),y.item()]
	print("roll: ", roll, r.item())
	print("pitch:", pitch, p.item())
	print("yaw:",yaw, y.item())
	print("all close?",np.allclose(base,res))

if __name__ == '__main__':
	main()
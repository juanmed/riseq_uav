import numpy as np

class KalmanFilter():
	"""Discrete Kalman Filter for linear system"""
	def __init__(self, A, B, H, Q, R, x0, dt = 1):
		
		self.A = A
		self.B = B
		self.H = H
		self.Q = Q
		self.R = R
		self.P = Q
		self.x = x0
		self.dt = dt

	def predict(self, u):
		self.x = np.dot(self.A, self.x) + np.dot(self.B,u)
		self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

	def update(self, z):
		S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

		if S.shape[0] > 1:
		  self.K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
		else:
		  self.K = np.dot(self.P, self.H.T / S)

		
		self.x = self.x + np.dot(self.K, z - np.dot(self.H, self.x))
		#self.P = self.P - np.dot(self.K, np.dot(S, self.K.T))
		self.P =  np.dot(np.eye(self.P.shape[0]) - np.dot(self.K, self.H), self.P)


	def filter(self, u, z):
		self.predict(u)
		self.update(z)
			

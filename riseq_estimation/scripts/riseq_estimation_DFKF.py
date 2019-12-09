import numpy as np

class KalmanFilter():
	"""Discrete Kalman Filter for linear system"""
	def __init__(self, A, B, H, D, Q, R, x0, dt = 1):
		
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
		self.P = np.dot(self.A, np.dot(self.P, A.T)) + self.Q

	def update(self, z):
		S = np.dot(H, np.dot(self.P, self.H.T)) + self.R
		self.K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
		self.x = self.x + np.dot(self.K, z - np.dot(self.H, self.x))
		self.P = self.P - np.dot(self.K, np.dot(S, self.K.T))

	def filter(self, u, z):
		self.predict(u)
		self.update(z)
		

#!/usr/bin/env python

import numpy as np

def hat3(v):
    """
    Hat operator
    """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def hat6(w, v):
    """
    Hat operator
    """
    A = np.zeros((4, 4))
    A[0:3, 0:3] = hat3(w)
    A[0:3, 3] = v
    return A


def vee3(V):
    """
    Vee operator
    """
    return np.array([[V[2][1]], [V[0][2]], [V[1][0]]])


def exp_so3(W):
    """
    Exponential map of so(3)
    """
    theta = np.sqrt(W[2][1]**2 + W[0][2]**2 + W[1][0]**2)
    return np.eye(3) + np.sin(theta)/theta*W + (1-np.cos(theta))/(theta**2)*np.dot(W, W)


def log_so3(R):
    """
    Logarithm map of SO(3)
    """
    theta = np.arccos((np.trace(R) - 1) / 2)
    return (R - R.T) * theta / (2*np.sin(theta))


def exp_se3(t):
    """
    Exponential map of se(3)
    """
    W = t[0:3, 0:3]
    theta = np.sqrt(W[2][1]**2 + W[0][2]**2 + W[1][0]**2)
    R = np.eye(3) + np.sin(theta)/theta*W + (1-np.cos(theta)/theta**2)*np.dot(W, W)
    V = np.eye(3) + (1-np.cos(theta))/(theta**2)*W + (theta-np.sin(theta))/(theta**3)*np.dot(W, W)
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = np.dot(V, t[0:3, 3])
    return T


def Adj_se3(T):
    """
    Adjoint of SE(3). (w v)
    """
    A = np.zeros((6, 6))
    A[0:3, 0:3] = T[0:3, 0:3]
    A[4:6, 4:6] = T[0:3, 0:3]
    A[4:6, 0:3] = np.dot(hat3(T[0:3, 3]), T[0:3, 0:3])
    return A


def adj_se3(w, v):
    """
    adjoint of se(3). (w v)
    """
    A = np.zeros((6, 6))
    A[0:3, 0:3] = hat3(w)
    A[4:6, 4:6] = hat3(w)
    A[4:6, 0:3] = hat3(v)
    return A


def r2q(R):
    """
    input: 3x3 rotation matrix
    output: quaternion angle
    """
    qw = np.sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
    qx = (R[2][1] - R[1][2]) / (4*qw)
    qy = (R[0][2] - R[2][0]) / (4*qw)
    qz = (R[1][0] - R[0][1]) / (4*qw)
    return qw, qx, qy, qz
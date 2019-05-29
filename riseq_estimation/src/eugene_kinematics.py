from math import sin, cos, tan, asin, acos, atan2
import numpy as np


def hat(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def rotation2quaternion(R):
    qw = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
    qx = (R[2][1] - R[1][2]) / (4*qw)
    qy = (R[0][2] - R[2][0]) / (4*qw)
    qz = (R[1][0] - R[0][1]) / (4*qw)
    return qw, qx, qy, qz


def rotation2euler(R):
    phi = atan2(R[2][1], R[2][2])
    theta = asin(-R[2][0])
    psi = atan2(R[1][0], R[0][0])
    return phi, theta, psi


def euler2quaternion(pi, theta, psi):
    qw = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2)
    qx = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)
    qy = sin(phi/2)*cos(theta/2)*sin(psi/2) + cos(phi/2)*sin(theta/2)*cos(psi/2)
    qz = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)
    return qw, qx, qy, qz

def pseudoInverseMatrixL(A):
    return np.dot(np.linalg.inv(np.dot(A.T, A)), A.T)

def pseudoInverseMatrixR(A):
    return np.dot(A.T, np.linalg.inv(np.dot(A, A.T)))
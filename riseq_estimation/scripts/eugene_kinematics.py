#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: This code is calculating general conversions or calculations for RISE-Q projects.
       Usually define conversions on Lie-group and various angles represent the state of a quadrotor.

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

from math import sin, cos, tan, asin, acos, atan2
import numpy as np


def hat(v):
    """
    Hat operator
    https://en.wikipedia.org/wiki/Hat_operator
    """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def rotation2quaternion(R):
    """
    Put 3x3 rotation matrix, then calculate quatrenion angle
    """
    qw = sqrt(1 + R[0][0] + R[1][1] + R[2][2]) / 2
    qx = (R[2][1] - R[1][2]) / (4*qw)
    qy = (R[0][2] - R[2][0]) / (4*qw)
    qz = (R[1][0] - R[0][1]) / (4*qw)
    return qw, qx, qy, qz


def rotation2euler(R):
    """
    Put 3x3 rotation matrix, then calculate Euler angles
    """
    phi = atan2(R[2][1], R[2][2])
    theta = asin(-R[2][0])
    psi = atan2(R[1][0], R[0][0])
    return phi, theta, psi


def euler2quaternion(pi, theta, psi):
    """
    Put Euler angles, then calculate quatrenion angle
    """
    qw = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2)
    qx = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2)
    qy = sin(phi/2)*cos(theta/2)*sin(psi/2) + cos(phi/2)*sin(theta/2)*cos(psi/2)
    qz = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2)
    return qw, qx, qy, qz


def q2r(qw, qx, qy, qz):
    """
    Put quaternion angle, then calculate rotation matrix
    """
    R = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qz*qx+2*qy*qw],
                  [2*qx*qy+2*qz*qw, 1-2*qz**2-2*qx**2, 2*qy*qz-2*qx*qw],
                  [2*qz*qx-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-w*qy**2]])
    return R


def pseudoInverseMatrixL(A):
    """ Left side pseudo inverse matrix """
    return np.dot(np.linalg.inv(np.dot(A.T, A)), A.T)


def pseudoInverseMatrixR(A):
    """ Right side pseudo inverse matrix """
    return np.dot(A.T, np.linalg.inv(np.dot(A, A.T)))
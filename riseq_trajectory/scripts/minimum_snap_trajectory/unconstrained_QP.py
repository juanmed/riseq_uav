###########################
# Reference
#
# [1]Charles Richter, Adam Bry and Nicholas Roy
# Polynomial Trajectory Planning for Quadrotor Flight
#
# [2]ARUN LAKSHMANAN,
# PIECEWISE BEZIER CURVE TRAJECTORY GENERATION AND CONTROL FOR QUADROTORS
###########################

from cvxopt import matrix, solvers
import numpy as np

import compute_matrix


def unconstrained_qp(order, waypoint, state, time):
    """
    Function to solve unconstrained quadratic programming.
     Traditional formulation becomes ill-conditioned for more than several segments,
     polynomials of high order, and when widely varying segment times are involved.
     It is only useful for short trajectory.
     In other words, inversion of matrices of that may be very close to singular.
     This reformulation is substantially more stable than the method above, allowing the joint optimization of more
     than 50 polynomial segments in a single matrix operation without encountering numerical issues[1].

     The inverse of matrix can be evaluated if the matrix is square. If not, psuedo inverse should be calculated[2].

    qp(P, q, None, None, A, b)
    min J = { 1/2 * c.T * P * c } ---------- c is array of polynomial of flat output [ x y z psi ].
    s.t. { A*c = b  }

    For formulating unconstrained_qp, it can't have inequality constraint.
    c = inv(A)*b
    min J = { 1/2 * b.T * inv(A).T * P * inv(A) * b }

    :param order: order of polynomial
    :param waypoint: point that drone need to traverse
    :param state: drone's state like velocity at end of each segment.
    :param time: segment allocated time
    :return: coefficient of polynomial and cost of function which is minimized by quadratic programming
    """
    # change format of way point
    # m : segment number
    # time scaling : time which is taken for each segment
    m = len(waypoint) - 1
    waypoint = np.transpose(waypoint)
    time = np.array(time)

    # fourth derivative for position, second derivative for yaw
    # These are relative to input T,M. Minimum snap equals minimum effort.
    # guarantee continuity until jerk and snap.
    k_r = 4
    k_psi = 2
    qp_matrix = compute_matrix.QpMatrix(order, m, waypoint, k_r, k_psi, time)

    # compute P, q
    mu_r = 1
    mu_psi = 1
    P = qp_matrix.compute_p(mu_r, mu_psi)
    q = matrix(0.0, (m * (order + 1) * 4, 1))
    # P = 2 * P

    # compute equality constraint: A,b
    A1, b1 = qp_matrix.waypoint_constraint()
    A2, b2 = qp_matrix.derivative_constraint(state)
    A3, b3 = qp_matrix.yaw_derivative_constraint(state)
    A = matrix([A1, A2, A3])
    b = matrix([b1, b2, b3])

    '''
     J = b.T * inv(A).T * P * inv(A) * b
     J = d.T * R * d    -------------------- d = [ dF dP ] R = [[RFF RFP],  F : fixed/specified derivatives
                                                                [RPF RPP]]  P : free/unspecified derivatives
     
    '''


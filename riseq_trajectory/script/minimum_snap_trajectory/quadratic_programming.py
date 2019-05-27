from cvxopt import matrix, solvers
import numpy as np

import compute_matrix


def qp_solution(order, waypoint, keyframe, current_state, time):
    """
    Function for calculating quadratic programming
    qp(P, q, G, h, A, b)
    min { 1/2 c.T * P * c + q * c } ---------- c is array of polynomial of flat output [ x y z psi ].
    G, b : Inequality Constraint ( Corridor Constraint, Minimum Maximum speed)
    A, b : Equality Constraint ( Way point Constraint, Derivative Constraint)
    """
    # change format of keyframe
    # m : segment number
    # time scaling : time which is taken for each segment
    keyframe = np.transpose(keyframe)
    m = waypoint - 1
    time = np.array(time)

    # fourth derivative for position, second derivative for yaw
    # These are relative to input T,M. Minimum snap equals minimum effort.
    # guarantee continuity until jerk and snap.
    k_r = 4
    k_psi = 2
    qp_matrix = compute_matrix.QpMatrix(order, m, keyframe, k_r, k_psi, time)

    # compute P, q
    mu_r = 1
    mu_psi = 1
    P = qp_matrix.compute_p(mu_r, mu_psi)
    q = matrix(0.0, (m * (order + 1) * 4, 1))
    P = 2 * P

    # compute equality constraint: A,b
    A1, b1 = qp_matrix.waypoint_constraint()
    A2, b2 = qp_matrix.derivative_constraint(current_state)
    A3, b3 = qp_matrix.yaw_derivative_constraint(current_state)
    A = matrix([A1, A2, A3])
    b = matrix([b1, b2, b3])

    # compute inequality constraint : G,h
    max_vel = 30
    min_vel = 0
    corridor_position = np.array([2, 3])
    corridor_width = 0.1
    n_intermediate = 3
    G1, h1 = qp_matrix.maxmin_constraint(max_vel, min_vel)
    #G2, h2 = qp_matrix.corridor_constraint(corridor_position, corridor_width, n_intermediate)
    #G = matrix([G1, G2])
    #h = matrix([h1, h2])

    sol = solvers.qp(P, q, G1, h1, A, b)
    sol_x = sol['x']
    val = sol['primal objective']

    # checking for when error occurs
    # print np.size(A)
    # print np.linalg.matrix_rank(A)
    # np.savetxt("A.csv", A, delimiter=",")
    # np.savetxt("b.csv", b, delimiter=",")
    # np.savetxt("G1.csv", G1, delimiter=",")
    # np.savetxt("h1.csv", h1, delimiter=",")
    # np.savetxt("G2.csv", G2, delimiter=",")
    # np.savetxt("h2.csv", h2, delimiter=",")
    # np.savetxt("x.csv", sol_x, delimiter=",")

    return sol_x, val

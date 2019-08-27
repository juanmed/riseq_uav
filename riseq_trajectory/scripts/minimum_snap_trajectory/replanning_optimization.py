###########################
# Reference
#
# [1]Charles Richter, Adam Bry and Nicholas Roy
# Polynomial Trajectory Planning for Quadrotor Flight
#
# [2]ARUN LAKSHMANAN,
# PIECEWISE BEZIER CURVE TRAJECTORY GENERATION AND CONTROL FOR QUADROTORS
#
# [3]Michael Burri, ETH Zurich
# Real-Time Visual-Inertial Mapping, Re-localization and Planning Onboard MAVs in Unknown Environments
###########################

import numpy as np
import compute_matrix_unQP
import time_optimal
from scipy.linalg import block_diag


def optimization(waypoint, time):
    """
     There are two cost functions.
    1. polynomial derivative cost function.
    It is for minimizing snap of position.
    For more detail, Look at the "unconstrained_QP.py".
    2. collision avoidance cost functions

    Jc = Integral{ c(f(t))ds } for S
       = Integral{ c(f(t)) ||v(t)|| dt } from 0 to tm
       = Sigma{ c(f(t))||v(t)||dt }

    Compute jacobian for each axis k [ x y z ], apply product and chain rule
    dJc/ddPk = sigma { ||v(t)|| gradient{c for k} T Lpp dt
                        + c(f(t))vk(t)/||v(t)||T V Lpp dt

    T = [t0 t1 t2 ... tN]
    fk(t) = Tpk , f(t) = [fx(t) fy(t) fz(t)]
    vk(t) = TVpk, v(t) = [vx(t) vy(t) vz(t)]

    invA * M = L = [Lff Lpp]

    c(x) = 1/2e(d(x) - e )^2
    d(x) = sqrt((fx(t) - X)^2 + (fy(t) - Y)^2 + (fz(t) - Z)^2)

    gradient(x) c = dC/dx = dC/dd dd/dx
    dC/dd = 1/e {d(x) - e}
    dd/dfx = 1/2 * {(fx-X)^2 + (fy-Y)^2 + (fz-Z)^2}^-0.5 * 2(fx - X)

    fx = Tpx = TL[dF dP]
    """
    waypoint = np.array([[0, 0, 0], [1, 5, 5], [20, 20, 20], [30, 30, 30]])
    m = len(waypoint) - 1
    time = np.ones(m)
    waypoint = np.transpose(waypoint)
    unqp = compute_matrix_unQP.UnConstraintQpMatrix(m, waypoint, 4, time)

    m = 5
    fixed_constraints = 5 * m + 5
    free_constraints = 10 * m - fixed_constraints

    P = unqp.compute_p_1axis_replanning(m)
    # convert to matrix for computing easily
    P = np.matrix(P)

    A = unqp.compute_A_1axis_replanning(m)

    start = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    goal = [[4, 0, 0, 0, 0], [4, 0, 0, 0, 0], [4, 0, 0, 0, 0]]
    b = unqp.end_derivative_replanning(m, start, goal)
    [dF, dP] = unqp.fixed_free_constraints_replanning(b, m)

    # dF is free constraint
    dF = np.transpose(dF)
    dF = np.matrix(dF)

    # dP is free constraint
    dP = np.ones((free_constraints, 3))
    dP = np.matrix(dP)

    d = np.zeros((10*m, 3))
    d = np.matrix(d)

    # d = [dF dP]
    for i in range(3):
        d[:, i] = np.vstack((dF[:, i], dP[:, i]))

    M = unqp.mapping_matrix_replanning(m)
    M = np.matrix(M)

    # compute every matrix which is used later
    invA = np.linalg.inv(A)
    R = M.T * invA.T * P * invA * M
    RFP = R[0:fixed_constraints, fixed_constraints:]
    RPP = R[fixed_constraints:, fixed_constraints:]
    invRPP = np.linalg.inv(RPP)
    L = invA * M
    Lpp = L[:, fixed_constraints:]

    # set initial guess
    d, dP = initial_guess(d, dF, dP, RFP, invRPP)

    # set parameter for collision cost function
    dt = 1
    threshold = 2
    obstacle = [0, 0, 1]

    # set parameter for gradient descent
    wd = 0.01
    wc = 1000
    rate = 0.0001
    tolerance = 0.1
    max_iteration = 100

    # cost value
    J = np.zeros(3)
    last_J = np.zeros(3)

    # gradient
    djd_ddp = np.zeros((free_constraints, 3))
    djd_ddp = np.matrix(djd_ddp)

    djc_ddp = np.zeros((free_constraints, 3))
    djc_ddp = np.matrix(djc_ddp)

    # dp = dp - rate * dJ_ddp
    # J = wd * Jd + wc * Jc

    # TODO
    """
     We choose to evaluate the function along every arc length point ds equal to the map voxel resoltion.
    This significantly speeds up computation without compromising safety
    """

    Jd, Jc = cost_value(P, invA, M, L, d, m, obstacle, threshold, dt)
    for i in range(3):
        J[i] = wd * Jd[i] + wc * Jc[i]

    for h in range(max_iteration):
        # iteration h for gradient descent

        djd_ddp, djc_ddp = gradient(RFP, RPP, L, Lpp, d, dF, dP, m, djd_ddp, djc_ddp, obstacle, threshold, dt)

        # update next dP
        for i in range(3):
            dP[:, i] = dP[:, i] - rate * (wd * djd_ddp[:, i] + wc * djc_ddp[:, i])

        # stack dF and new dP
        for i in range(3):
            d[:, i] = np.vstack((dF[:, i], dP[:, i]))

        Jd, Jc = cost_value(P, invA, M, L, d, m, obstacle, threshold, dt)

        for i in range(3):
            J[i] = wd * Jd[i] + wc * Jc[i]

        if abs(last_J[0] - J[0]) < tolerance or abs(last_J[1] - J[1]) < tolerance or abs(last_J[2] - J[2]) < tolerance:
            break
        for i in range(3):
            last_J[i] = J[i]

    solution = np.zeros((10 * m, 3))
    invA = np.linalg.inv(A)
    for i in range(3):
        solution[:, i] = (invA * M * d[:, i]).flatten()
    plot_traj3D(solution.T, 9, m, None, 0)
    time = np.ones(m) * 10/m

    time = time_optimal.time_optimal(solution, m)
    print d
    print time


def get_vector(k, t):
    """
     k-th derivative at time t
    """
    order = 9
    compute_mat = np.eye(order + 1)
    values = np.zeros(order + 1)
    for j in range(0, order + 1):
        tempCoeffs = compute_mat[j, :]
        for i in range(0, k):
            tempCoeffs = np.polyder(tempCoeffs)
        values[j] = np.polyval(tempCoeffs, t)
    return values


def initial_guess(d, dF, dP, RFP, invRPP):
    """
     Function to set initial parameter of free constraints.
    Set free constraints using theory of unconstrained QP without collision cost function.
    """

    for i in range(3):
        dP[:, i] = -invRPP * RFP.T * dF[:, i]
        d[:, i] = np.vstack((dF[:, i], dP[:, i]))

    return d, dP


def gradient(RFP, RPP, L, Lpp, d, dF, dP, m, djd_ddp, djc_ddp, obstacle, threshold, dt):
    """
     Function to compute gradient
    """
    for i in range(3):
        # gradient of first cost function
        djd_ddp[:, i] = (2 * dF[:, i].T * RFP + 2 * dP[:, i].T * RPP).T

    for i in range(3):
        summation_jacobian = np.zeros(5 * m - 5)

        for j in range(m + 1):
            if j == m:
                P_vector = np.zeros(m * 10)
                V_vector = np.zeros(m * 10)
                P_vector[(m - 1) * 10:] = get_vector(0, 1)
                V_vector[(m - 1) * 10:] = get_vector(1, 1)
            else:
                P_vector = np.zeros(m * 10)
                V_vector = np.zeros(m * 10)
                P_vector[j * 10: j * 10 + 10] = get_vector(0, 0)
                V_vector[j * 10: j * 10 + 10] = get_vector(1, 0)

            vk = np.zeros(3)
            fk = np.zeros(3)
            for k in range(3):
                vk[k] = (np.matrix(V_vector) * L * d[:, k]).item(0)
                fk[k] = (np.matrix(P_vector) * L * d[:, k]).item(0)
            norm_velocity = np.linalg.norm((vk[0], vk[1], vk[2]))
            distance = np.linalg.norm((fk[0] - obstacle[0], fk[1] - obstacle[1], fk[2] - obstacle[2]))

            if distance <= threshold:
                dc_dd = 1.0 / threshold * (distance - threshold)
                dd_df = 1.0 / 2 * np.power(
                    ((fk[0] - obstacle[0]) ** 2 + (fk[1] - obstacle[1]) ** 2 + (fk[2] - obstacle[2]) ** 2),
                    -0.5) * 2 * (fk[i] - obstacle[i])
                c_f = 1.0 / (2 * threshold) * (distance - threshold) ** 2
            else:
                dc_dd = 0
                dd_df = 0
                c_f = 0
            VL = np.matrix(V_vector) * Lpp
            PL = np.matrix(P_vector) * Lpp
            sum1 = PL * norm_velocity * dc_dd * dd_df * dt

            if norm_velocity == 0 or norm_velocity == 0.0:
                sum2 = VL * c_f * vk[i] / 0.001 * dt
            else:
                sum2 = VL * c_f * vk[i] / norm_velocity * dt
            sum2 = sum2

            summation_jacobian = sum1 + sum2 + summation_jacobian

        # gradient of second cost function
        djc_ddp[:, i] = summation_jacobian.T

    return djd_ddp, djc_ddp


def cost_value(P, invA, M, L, d, m, obstacle, threshold, dt):
    """
     Function to compute cost value
    """

    Jd = np.zeros(3)
    Jc = np.zeros(3)

    for i in range(3):
        # First cost value
        p = invA * M * d[:, i]
        Jd[i] = (p.T * P * p).item(0)

    for i in range(3):
        # Second cost value
        summation_cost = 0

        for j in range(m + 1):
            if j == m:
                P_vector = np.zeros(m * 10)
                V_vector = np.zeros(m * 10)
                P_vector[(m - 1) * 10:] = get_vector(0, 1)
                V_vector[(m - 1) * 10:] = get_vector(1, 1)
            else:
                P_vector = np.zeros(m * 10)
                V_vector = np.zeros(m * 10)
                P_vector[j * 10: j * 10 + 10] = get_vector(0, 0)
                V_vector[j * 10: j * 10 + 10] = get_vector(1, 0)

            vk = np.zeros(3)
            fk = np.zeros(3)
            for k in range(3):
                vk[k] = (np.matrix(V_vector) * L * d[:, k]).item(0)
                fk[k] = (np.matrix(P_vector) * L * d[:, k]).item(0)
            norm_velocity = np.linalg.norm((vk[0], vk[1], vk[2]))
            distance = np.linalg.norm((fk[0] - obstacle[0], fk[1] - obstacle[1], fk[2] - obstacle[2]))

            if distance <= threshold:
                c_f = 1.0 / (2 * threshold) * (distance - threshold) ** 2
            else:
                c_f = 0
            summation_cost = c_f * norm_velocity * dt + summation_cost
            Jc[i] = summation_cost
    return Jd, Jc


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_traj3D(solution, order, m, keyframe, k):
    """
    Plot trajectory of k th derivative from 3 dimension [ x y z ] solution
    """

    n = 3
    x_trajec = []
    y_trajec = []
    z_trajec = []

    for i in range(0, m):
        # we can use np.arrange instead of np.linspace
        x_trajec = np.append(x_trajec, np.polyval(
            solution[0][i * (order + 1): i * (order + 1) + (order + 1)],
            np.linspace(0, 1, 50)))
        y_trajec = np.append(y_trajec, np.polyval(
            solution[1][i * (order + 1): i * (order + 1) + (order + 1)],
            np.linspace(0, 1, 50)))
        z_trajec = np.append(z_trajec, np.polyval(
            solution[2][i * (order + 1): i * (order + 1) + (order + 1)],
            np.linspace(0, 1, 50)))

    # plot x y z
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title('xyz')
    ax.plot(x_trajec, y_trajec, z_trajec, 'r')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    if keyframe is not None:
        for i in range(0, len(keyframe)):
            ax.text(keyframe[i][0], keyframe[i][1], keyframe[i][2], i, color='red')
    plt.show()


def plot_snap3D(solution, order, m, time_scaling):
    x_snap = []
    y_snap = []
    z_snap = []
    n = 3

    for i in range(0, m):
        time = np.linspace(0, 1 / time_scaling[i] ** 2, 50)
        factor = np.linspace(0, 1 / time_scaling[i] ** 2, 50)
        # we can use np.arrange instead of np.linspace
        x_snap = np.append(x_snap, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 0 * (order + 1): i * n * (order + 1) + (order + 1) + 0 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))
        y_acc = np.append(y_acc, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 1 * (order + 1): i * n * (order + 1) + (order + 1) + 1 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))
        z_acc = np.append(z_acc, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 2 * (order + 1): i * n * (order + 1) + (order + 1) + 2 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))
        psi_acc = np.append(psi_acc, np.polyval(np.polyder(
            solution[i * n * (order + 1) + 3 * (order + 1): i * n * (order + 1) + (order + 1) + 3 * (order + 1)], 1),
            np.linspace(0, 1 / time_scaling[i]**2, 50)))

        # plot x y z
        fig = plt.figure(2)
        axx = fig.add_subplot(211)
        axy = fig.add_subplot(212)
        axz = fig.add_subplot(213)
        axpsi = fig.add_subplot(214)
        axx.set_title('x_acc')
        axy.set_title('y_acc')
        axz.set_title('z_acc')
        axpsi.set_title('psi_acc')
        axx.set_xlim(-30, 30)
        axy.set_ylim(-30, 30)
        axz.set_zlim(-30, 30)
        axpsi.set_zlim(-3.14, 3.14)

if __name__ == "__main__":
    waypoint = np.array([[0, 0, 0], [1, 5, 5], [20, 20, 20], [30, 30, 30]])
    m = len(waypoint) - 1
    time = np.ones(m)
    optimization(waypoint, time)

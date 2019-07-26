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

from cvxopt import matrix
import numpy as np
import compute_matrix_unQP


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

    P = unqp.compute_p_1axis_replanning(5)
    P = matrix(P)

    A = unqp.compute_A_1axis_replanning(5)

    m = 5
    start = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    goal = [[4, 0, 0, 0, 0], [4, 0, 0, 0, 0], [7, 0, 0, 0, 0]]
    b = unqp.end_derivative_replanning(5, start, goal)
    [bF, bP] = unqp.fixed_free_constraints_replanning(b, 5)

    dF = np.array(bF)
    dP = np.array(bP)

    # dP is free constraint
    # initial guess
    '''
    dP = np.ones((3, 5 * m - 5))
    for i in range(3):
        intermediate = np.linspace(start[i][0], goal[i][0], m + 1)
        for j in range(m-1):
            dP[i][j*5] = intermediate[j+1]
    '''
    # d = [dF dP]
    d = np.zeros((3, 10 * m))
    for i in range(3):
        d[i] = np.hstack((dF[i], dP[i]))

    M = unqp.mapping_matrix_replanning(5)
    M = matrix(M)
    np.savetxt("A.csv", A, delimiter=",")
    np.savetxt("M.csv", M, delimiter=",")
    invA = matrix(np.linalg.inv(A))
    R = M.T * invA.T * P * invA * M

    np.savetxt("R.csv", R, delimiter=",")
    RFP = R[0:(5 * m + 5), (5 * m + 5):]
    RPP = R[(5 * m + 5):, (5 * m + 5):]

    invRPP = matrix(np.linalg.inv(RPP))
    X = -invRPP * RFP.T
    solution = np.zeros((3, 10 * m))
    for i in range(3):
        dP[i][:] = np.array(X * matrix(dF[i])).T
        d[i] = np.hstack((dF[i], dP[i]))
        p = invA * M * matrix(d[i])

        solution[i][:] = np.array(p.T)
        # value of cost function which is minimized by optimization
        cost = p.T * P * p

    print d[0]
    print M * matrix(d[0])
    print dF
    print dP
    plot_traj3D(solution, 9, 5, None)

    L = invA * M
    Lpp = L[:, (5 * m + 5):]

    djd_ddp = np.zeros((3, 5 * m - 5))
    djc_ddp = np.zeros((3, 5 * m - 5))
    Jd = np.zeros(3)
    Jc = np.zeros(3)
    J = np.zeros(3)
    wd = 1
    wc = 1
    dt = 1
    e = 3
    obstacle = [2, 2, 2]

    rate = 0.01

    # J = wd * Jd + wc * Jc
    # dp = dp - rate * dJ_ddp

    solution = np.zeros((3, 10 * m))
    for h in range(4):
        # iteration h for gradient descent

        for i in range(3):
            # First cost function Jd
            d[i] = np.hstack((dF[i], dP[i]))
            p = invA * M * matrix(d[i])
            solution[i][:] = np.array(p.T)
            Jd[i] = (p.T * P * p)[0]
            djd_ddp[i] = 2 * matrix(-dF[i]).T * RFP + 2 * matrix(dP[i]).T * RPP

        for k in range(3):
            # Second cost function Jc
            summation_cost = 0
            summation = np.zeros(5 * m - 5)
            for i in range(m + 1):
                if i == m:
                    P_vector = np.zeros(m * 10)
                    V_vector = np.zeros(m * 10)
                    P_vector[(m - 1) * 10:] = get_vector(0, 1)
                    V_vector[(m - 1) * 10:] = get_vector(1, 1)
                else:
                    P_vector = np.zeros(m * 10)
                    V_vector = np.zeros(m * 10)
                    P_vector[i * 10: i * 10 + 10] = get_vector(0, 0)
                    V_vector[i * 10: i * 10 + 10] = get_vector(1, 0)

                vk = np.zeros(3)
                fk = np.zeros(3)
                for j in range(3):
                    vk[j] = (matrix(V_vector).T * L * matrix(d[j]))[0]
                    fk[j] = (matrix(P_vector).T * L * matrix(d[j]))[0]
                norm_velocity = np.linalg.norm((vk[0], vk[1], vk[2]))
                distance = np.linalg.norm((fk[0] - obstacle[0], fk[1] - obstacle[1], fk[2] - obstacle[2]))
                # print distance
                if distance <= e:
                    dc_dd = 1.0 / e * (distance - e)
                    dd_df = 1.0 / 2 * np.power(
                        ((fk[0] - obstacle[0]) ** 2 + (fk[1] - obstacle[1]) ** 2 + (fk[2] - obstacle[2]) ** 2),
                        -0.5) * 2 * (fk[k] - obstacle[k])
                    c_f = 1.0 / (2 * e) * (distance - e) ** 2
                else:
                    dc_dd = 0
                    dd_df = 0
                    c_f = 0
                VL = matrix(V_vector).T * Lpp
                PL = matrix(P_vector).T * Lpp
                sum1 = PL * norm_velocity * dc_dd * dd_df * dt
                if norm_velocity == 0 or norm_velocity == 0.0:
                    sum2 = VL * c_f * vk[k] / 0.001 * dt
                else:
                    sum2 = VL * c_f * vk[k] / norm_velocity * dt

                summation = sum1 + sum2 + summation
                summation_cost = c_f * norm_velocity * dt + summation_cost

            #djc_ddp[k] = summation
            Jc[k] = summation_cost

        for k in range(3):
            J[k] = wd * Jd[k] + wc * Jc[k]

        # update dP
        for i in range(3):
            dP[i] = dP[i] - rate * wd * djd_ddp[i] - rate * wc * djc_ddp[i]

        #print dP[0]
        # print J

    #print solution
    plot_traj3D(solution, 9, 5, None)


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


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_traj3D(solution, order, m, keyframe):
    """
    Plot trajectory from 3 dimension [ x y z ] solution
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


if __name__ == "__main__":
    waypoint = np.array([[0, 0, 0], [1, 5, 5], [20, 20, 20], [30, 30, 30]])
    m = len(waypoint) - 1
    time = np.ones(m)
    optimization(waypoint, time)

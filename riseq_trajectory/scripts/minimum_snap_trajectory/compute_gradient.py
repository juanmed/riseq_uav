import numpy as np
from scipy.optimize import newton


def derivative_free_gradient(RFP, RPP, dF, dP, djd_ddp):
    """
     Function to compute gradient of derivative cost function
    This cost function has two variables as free constraint and segment time

    derivative_cost_function / free_constraint
    dJd / ddp

    dJd/ddp = 2dF.T * RFP + 2dP.T * RPP

    [free constraints x 3] matrix form
    """
    for i in range(3):
        # gradient of first cost function
        djd_ddp[:, i] = (2 * dF[:, i].T * RFP + 2 * dP[:, i].T * RPP).T

    return djd_ddp


def derivative_time_gradient(order, k_r, time, m, c):
    """
     Function to compute gradient of derivative cost function
    This cost function has two variables as free constraint and segment time

    J = Jd + kT * ( sum(T))

    derivative_cost_function / segment_time
    dJd / dT

    Jd = (p.T * Q * p) * 1/time**7
    dJd/dT = (p.T * Q * p) * -7/time**8

    [ 1 x m ] vector form
    """
    dJd_dT = np.zeros(m)

    # compute P
    polynomial_r = np.ones(order + 1)
    for i in range(0, k_r):
        polynomial_r = np.polyder(polynomial_r)

    p = np.zeros((order + 1, order + 1))
    for j in range(0, order + 1):
        for k in range(j, order + 1):
            # position
            if j <= len(polynomial_r) - 1 and k <= len(polynomial_r) - 1:
                order_t_r = ((order - k_r - j) + (order - k_r - k))
                if j == k:
                    p[j, k] = np.power(polynomial_r[j], 2) / (order_t_r + 1)
                else:
                    p[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1)

    p = p + np.transpose(p)
    p = 0.5 * p
    p = np.matrix(p)

    for i in range(m):
        dJd_dT[i] = np.sum(c[i * (order + 1): i * (order + 1) + order + 1, j].T * p * c[i * (order + 1): i * (order + 1) + order + 1, j] for j in range(3))
        dJd_dT[i] = dJd_dT[i] * -8 / time[i]**7

    return dJd_dT


def constrained_time(dJd_dT, m):
    """
    Constrained Gradient Descent Method

     ||a|| : constant
    ex) a1 + a2 + a3 ... + am = 10

    normal vector n = < 1, 1, 1 ... 1>
    tangential vector t = gradient J - ( gradient J * n )n

    a = a - r * t
    """
    # normal vector of time plane
    n = np.ones(m)
    n = n / np.linalg.norm(n)

    # tangential vector of time plane
    t = dJd_dT - np.dot(dJd_dT, n) * n

    return t


def time_gradient(m):
    """
     Function to compute gradient of time cost function

    Jt = kt { time[1] ... time[m] }
    dJt/dT = kt { 1 ... 1 }

    [ 1 x m ] vector form
    """
    dJt_dT = np.ones(m)
    return dJt_dT


def collsion_gradient(L, Lpp, d, m, djc_ddp, obstacle, threshold, dt, free_constraint):
    """
     Function to compute gradient of collision cost function
    This has only one variable as free constraint

    Sigma[ c(F(T)) * ||V(T)|| * dT ] = sigma[ c(f(t)) * ||v(t)/a|| * a dt ] = sigma[ c(f(t)) * ||v(t)|| * dt ]
    f(t), v(t) is independent on segment time.

    Compute gradient for each axis k [ x y z ], apply product and chain rule
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

    [free constraints x 3] matrix form
    """
    # N = np.arange(0, time, dt)
    # N = np.linspace(0, time, n)

    # If dt == 1, T = [0]
    T = np.arange(0, 1, dt)  # * time_scaling

    for i in range(3):
        summation_jacobian = np.zeros(free_constraint)

        for j in range(m):
            for t in T:
                P_vector = np.zeros(m * 10)
                V_vector = np.zeros(m * 10)
                P_vector[j * 10: j * 10 + 10] = get_vector(0, t)
                V_vector[j * 10: j * 10 + 10] = get_vector(1, t)

                vk = np.zeros(3)  # /time_scaling
                fk = np.zeros(3)
                for k in range(3):
                    vk[k] = (np.matrix(V_vector) * L * d[:, k]).item(0)
                    fk[k] = (np.matrix(P_vector) * L * d[:, k]).item(0)
                norm_velocity = np.linalg.norm((vk[0], vk[1], vk[2]))
                distance = np.linalg.norm((fk[0] - obstacle[0], fk[1] - obstacle[1], fk[2] - obstacle[2]))

                VL = np.matrix(V_vector) * Lpp
                PL = np.matrix(P_vector) * Lpp
                if distance <= threshold:
                    dc_dd = 1.0 / threshold * (distance - threshold)
                    dd_df = 1.0 / distance * (fk[i] - obstacle[i])
                    c_f = 1.0 / (2 * threshold) * (distance - threshold) ** 2
                    sum1 = norm_velocity * dc_dd * dd_df * PL * dt  # not depend on time scaling
                else:
                    # distance > threshold
                    c_f = 0
                    sum1 = 0

                if norm_velocity == 0:
                    # vk is also 0
                    sum2 = 0
                else:
                    sum2 = VL * c_f * vk[i] / norm_velocity * dt  # not depend on time scaling

                summation_jacobian = sum1 + sum2 + summation_jacobian

        # gradient of second cost function
        djc_ddp[:, i] = summation_jacobian.T

    return djc_ddp


def endpoint_free_gradient(dJe_ddp, L, Lpp, d, m, pep, vep, lamda1, lamda2):
    """
     Function to compute gradient of end-point cost function relative to free constraints

    dJe_ddp = lamda1 * 2 * (ptep - pep) * P * Lpp + lamda2 * 2 * (vtep - vep) * V * Lpp

    [free constraints x 3] matrix form
    """
    for i in range(3):
        P_vector = np.zeros(m * 10)
        V_vector = np.zeros(m * 10)
        P_vector[(m - 1) * 10:] = get_vector(0, 1)
        V_vector[(m - 1) * 10:] = get_vector(1, 1)

        ptep = (np.matrix(P_vector) * L * d[:, i]).item(0)
        vtep = (np.matrix(V_vector) * L * d[:, i]).item(0)

        dJe_ddp[i] = lamda1 * 2 * (ptep - pep[i]) * P_vector * Lpp + lamda2 * 2 * (vtep - vep[i]) * V_vector * Lpp
    return dJe_ddp


def maxt(t, coefficient):
    length = len(coefficient)
    poly = np.sum(coefficient[i] * t**(length - 1 - i) for i in range(length))
    return poly


def soft_constraint_gradient(p, dJs_ddp, L, Lpp, d, m, vel_max, acc_max, free_constraint):
    """
     Function to compute gradient of soft-constraint cost function relative to free constraints

    For k axis,
    dJe_ddp = Sigma[ exp(vel_norm - vel_max) / vel_norm * vk * V * Lpp + exp(acc_norm - acc_max) /acc_norm * ak * A * Lpp ]

    [free constraints x 3] matrix form
    """

    v_actual = np.zeros(3)  # / time_scaling
    a_actual = np.zeros(3)  # / time_scaling**2

    v_coefficient = np.zeros((10, 3))
    a_coefficient = np.zeros((10, 3))
    j_coefficient = np.zeros((10, 3))

    summation_jacobian = np.zeros((free_constraint, 3))
    summation_jacobian = np.matrix(summation_jacobian)

    for i in range(m):
        V_vector = np.zeros(m * 10)
        A_vector = np.zeros(m * 10)

        for j in range(3):
            v_coefficient[:, j] = p[i * 10: i * 10 + 10, j] * get_vector(1, 1)
            a_coefficient[:, j] = p[i * 10: i * 10 + 10, j] * get_vector(2, 1)
            j_coefficient[:, j] = p[i * 10: i * 10 + 10, j] * get_vector(3, 1)

        v_norm_dot = np.sum(np.poly1d(v_coefficient[:, i]) * np.poly1d(a_coefficient[:, i]) for i in range(3))  # / v_norm
        v_norm_dot = np.array(v_norm_dot)

        a_norm_dot = np.sum(np.poly1d(a_coefficient[:, i]) * np.poly1d(j_coefficient[:, i]) for i in range(3))  # / a_norm
        a_norm_dot = np.array(a_norm_dot)

        vel_t = newton(maxt, 0.5, args=(v_norm_dot,))
        acc_t = newton(maxt, 0.5, args=(a_norm_dot,))

        V_vector[i * 10: i * 10 + 10] = get_vector(0, vel_t)
        A_vector[i * 10: i * 10 + 10] = get_vector(1, acc_t)

        for j in range(3):
            v_actual[j] = (np.matrix(V_vector) * L * d[:, j]).item(0)  # / time_scaling
            a_actual[j] = (np.matrix(A_vector) * L * d[:, j]).item(0)  # / time_scaling**2

        vel_norm = np.linalg.norm(v_actual)
        acc_norm = np.linalg.norm(a_actual)

        VL = np.matrix(V_vector) * Lpp
        AL = np.matrix(A_vector) * Lpp

        for j in range(3):
            summation_jacobian[:, j] = summation_jacobian[:, j] + np.exp(vel_norm - vel_max) / vel_norm * v_actual[j] * VL + \
                                       np.exp(acc_norm - acc_max) / acc_norm * a_actual[j] * AL

    for j in range(3):
        dJs_ddp[:, j] = summation_jacobian[:, j]

    return dJs_ddp


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



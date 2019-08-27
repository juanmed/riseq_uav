import numpy as np
from scipy.optimize import newton


def derivative_cost(P, invA, M, d):
    """
     compute cost of derivative cost function

    Each axis is independent [x y z]

    [ 1 x 3 ] vector

    1/time_scaling ** 7
    """
    Jd = np.zeros(3)

    for i in range(3):
        p = invA * M * d[:, i]
        Jd[i] = (p.T * P * p).item(0)

    return Jd


def time_cost(time):
    """
     compute cost of derivative cost function

    scalar
    """
    Jt = np.sum(time)
    return Jt


def collision_cost(L, d, m, obstacle, threshold, dt):
    """
     compute cost of collision cost function

    Jc = Integral{ c(f(t))ds } for S
       = Integral{ c(f(t)) ||v(t)|| dt } from 0 to tm
       = Sigma{ c(f(t))||v(t)||dt }

    [ 1 x 3 ] vector
    """
    Jc = np.zeros(3)

    # If dt == 1, T = [0]
    T = np.arange(0, 1, dt)  # * time_scaling

    for i in range(3):
        # Second cost value
        summation_cost = 0

        for j in range(m):
            for t in T:
                P_vector = np.zeros(m * 10)
                V_vector = np.zeros(m * 10)
                P_vector[j * 10: j * 10 + 10] = get_vector(0, t)
                V_vector[j * 10: j * 10 + 10] = get_vector(1, t)

                vk = np.zeros(3)  # / time_scaling
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
                summation_cost = c_f * norm_velocity * dt + summation_cost  # not depend on time scaling
        Jc[i] = summation_cost
    return Jc


def end_point_cost(L, d, m, pep, vep, lamda1, lamda2):
    """
     compute cost value of Endpoint cost function

    Je = lamda1(p(tep) - pep)^2 + lamda2(v(tep) - vep)^2
    pep, vep is end position and velocity of global trajectory.
    p(tep), v(tep) is end position and velocity of local trajectory.
    This function is for minimizing error between end of global and local trajectory.

    [ 1 x 3 ] vector
    """
    Je = np.zeros(3)
    for i in range(3):
        P_vector = np.zeros(m * 10)
        V_vector = np.zeros(m * 10)
        P_vector[(m-1) * 10:] = get_vector(0, 1)
        V_vector[(m-1) * 10:] = get_vector(1, 1)

        ptep = (np.matrix(P_vector) * L * d[:, i]).item(0)
        vtep = (np.matrix(V_vector) * L * d[:, i]).item(0)  # / time_scaling

        Je[i] = lamda1 * (ptep - pep[i])**2 + lamda2 * (vtep - vep[i])**2
    return Je


def maxt(t, coefficient):
    length = len(coefficient)
    poly = np.sum(coefficient[i] * t**(length - 1 - i) for i in range(length))
    return poly


def soft_constraint_cost(p, L, d, m, vel_max, acc_max):
    """
     Compute cost value of soft-constraint function

    Js = sigma [l(x)]
    l(x) = exp[ x - xmax ] - 1             if x > xmax
         = 0                               if x < xmax

    v_norm = sqrt [ vx^2 + vy^2 + vz^2 ]
    v_norm' = vx * ax + vy * ay + vz * az / v_norm
    Maximum velocity when v_norm' = 0

     Inequality constraint increases the number of necessary iterations significantly and the optimizer does not always
    respect the constraints. Since state constraints are more guidelines than hard constraints in our constraints,
    we implemented state constraints as soft constraints by adding an additional cost term.

    scalar
    """
    p = np.array(p)

    Js = np.zeros(m)

    v_actual = np.zeros(3)  # / time_scaling
    a_actual = np.zeros(3)  # / time_scaling**2

    v_coefficient = np.zeros((10, 3))
    a_coefficient = np.zeros((10, 3))
    j_coefficient = np.zeros((10, 3))

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

        Js[i] = np.exp(acc_norm - acc_max) + np.exp(vel_norm - vel_max)

    return np.sum(Js)


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

import numpy as np


def go_upward(order, time, distance):
    """
     This function is to make polynomial which just go upward
    In equation, [x, y, z, psi] = [0, 0, f(t), 0]
    Because drone moves along z-axis, other variable must be zero.
    Except final position, all state is zero

    :param order: In this case, 6 constraints exist.
    Initial and final position, velocity, acceleration.
    Able to determine 6 coefficient... z(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
    order = 5
    :param time: Time to take to move.
    :param distance: Distance to move
    :return: coefficient of [ 0, 0, z(t), 0 ]
    """

    # Construct A Matrix and b Vector with 6 Constraint
    A = np.zeros((order+1, order+1))
    for i in range(0, 3):
        for j in range(0, 2):
            A[2*i+j] = poly_cc(order, i, j*time)
    b = np.zeros(order+1)
    b[1] = distance

    # Solution: z(t) = InvA * b
    sol_z = np.linalg.solve(A, b)

    # Other Solution: x(t), y(t), psi(t) = 0
    # Construct Solution form for differential flatness
    # Solution form : [ 0, 0, z(t), 0 ]
    other_sol = np.zeros(order+1)
    solution = np.hstack((other_sol, other_sol, sol_z, other_sol))
    return solution


def go_along(order, time, final_point):
    """
     This function is to make polynomial which connect initial point with desired coordinate

    :param order: In this case, 6 constraints exist.
    Initial and final position, velocity, acceleration.
    Able to determine 6 coefficient... x(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
    order = 5
    :param time: Time to take to move.
    :param final_point: [ desired_x, desired_y, desired_z, desired_psi ]
    :return: coefficient of [ x(t), y(t), z(t), psi(t) ]
    """
    solution = np.zeros(4 * (order + 1))
    # Construct A Matrix and b Vector with 6 Constraint
    for h in range(0, 4):
        A = np.zeros((order+1, order+1))
        for i in range(0, 3):
            for j in range(0, 2):
                A[2*i+j] = poly_cc(order, i, j*time)
        b = np.zeros(order+1)
        b[1] = final_point[h]
        # Solution: z(t) = InvA * b
        solution[h*(order+1): (h+1)*(order+1)] = np.linalg.solve(A, b)

    # Construct Solution form for differential flatness
    # Solution form : [ x(t), y(t), z(t), psi(t) ]
    return solution


def poly_cc(order, k, t):
    """
    Helper function to get coefficient of k th derivative at time t from nth order polynomial.
    """
    value = np.zeros(order+1)
    # Matrix for computation
    compute_mat = np.eye(order+1)
    for i in range(0, order+1):
        value[i] = np.polyval(np.polyder(compute_mat[i], k), t)
    return value

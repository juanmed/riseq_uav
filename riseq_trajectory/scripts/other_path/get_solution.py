import numpy as np


m = 0 # segment
order = 7 # order of polynomial
n = 4 # output


def get_solution(time, position, velocity, acceleration):
    """
         This function is to make polynomial which connect way point and satisfy derivative continuity
        The order of polynomial must be 7th order.
        Reason is like this.
        At first, we need 6 constraints at each segment: Initial and final position, velocity, acceleration.
        Moreover, we must consider continuity for jerk and snap.
        However, First polynomial need initial jerk as 0 and don't need snap constraint.
        Similarly, Last polynomial need final jerk as 0 and don't need snap constraint.

        In conclusion, the number of constraint is {6*segment + 2*(segment-1) + 2} = 8*segment
        So we need 8 coefficient at each segment to determine polynomial.

        Able to determine 8 coefficient... x(t) = at^7 + bt^6 + ct^5 + dt^4 + et^3 + ft^2 + gt + h
        :param time: Time to take to move.
        :param keyframe: [ desired_x, desired_y, desired_z, desired_psi ]
        :return: coefficient of [ x(t), y(t), z(t), psi(t) ]
    """

    # position is [ x(t), y(t), z(t), psi(t) ]
    global m
    m = len(time) # segment

    if type(velocity) == int or type(velocity) == float:
        velocity = np.ones((m+1, n)) * velocity
    if type(acceleration) == int or type(acceleration) == float:
        velocity = np.ones((m+1, n)) * acceleration

    print velocity
    # keyframe is consist of position, velocity, acceleration
    # array { 3 x m x n }
    keyframe = np.array((position, velocity, acceleration))

    # Construct A Matrix and b vector with Constraint
    A1 = np.zeros((n * 6 * m, n * (order + 1) * m))
    b1 = np.zeros(n * 6 * m)

    # Construct A Matrix and b Vector with Constraint
    for i in range(0, m):
        for j in range(0, n):
            for k in range(0, 3):
                a = np.zeros(n * (order + 1) * m)
                a[i * (order + 1) * n + j * (order + 1):
                  i * (order + 1) * n + j * (order + 1) + order + 1] = poly_cc(order, k, 0)
                A1[i * n * 6 + j * 6 + k, :] = a
                b1[i * n * 6 + j * 6 + k] = keyframe[k, i, j]

                a = np.zeros(n * (order + 1) * m)
                a[i * (order + 1) * n + j * (order + 1):
                  i * (order + 1) * n + j * (order + 1) + order + 1] = poly_cc(order, k, time[i])
                A1[i * n * 6 + j * 6 + k + 3, :] = a
                b1[i * n * 6 + j * 6 + k + 3] = keyframe[k, i+1, j]

    A2, b2 = construct_continuity(time)
    A = np.vstack((A1, A2))
    b = np.hstack((b1, b2))

    # Solution: z(t) = InvA * b
    solution = np.linalg.solve(A, b)
    #np.savetxt("solution.csv", solution, delimiter=",")

    # solution form is like this : [ x1(t), y1(t), z1(t), psi1(t), x2(t), y2(t), z2(t), psi2(t), ...]
    return solution

def construct_continuity(time):
    """
        This function is to make A matrix to build constraint for jerk and snap.
    """
    A2 = np.zeros((n * 2 * m, n * (order + 1) * m))
    b2 = np.zeros(n * 2 * m)
    for i in range(0, m):
        for j in range(0, n):
            if i == 0:
                # only jerk continuity at start and finish point
                values = poly_cc(order, 3, 0)
                a = np.zeros(n * (order + 1) * m)

                a[i * (order + 1) * n + j * (order + 1):
                  i * (order + 1) * n + j * (order + 1) + order + 1] = values
                A2[j, :] = a

                values = poly_cc(order, 3, time[i])
                a = np.zeros(n * (order + 1) * m)
                a[(m - 1) * (order + 1) * n + j * (order + 1):
                  (m - 1) * (order + 1) * n + j * (order + 1) + order + 1] = values
                A2[n + j, :] = a

            else:
                for k in range(0, 2):
                    # jerk, snap continuity
                    start_values = poly_cc(order, k+3, 0)
                    end_values = poly_cc(order, k+3, time[i-1])
                    a = np.zeros(n * (order + 1) * m)
                    a[(i-1) * (order + 1) * n + j * (order + 1):
                      (i-1) * (order + 1) * n + j * (order + 1) + order + 1] = end_values
                    a[i * (order + 1) * n + j * (order + 1):
                      i * (order + 1) * n + j * (order + 1) + order + 1] =  - start_values
                    A2[2 * i * n + 2 * j + k, :] = a

    return A2, b2

def poly_cc(order, k, t):
    """
        Helper function to get coefficient of k th derivative at time t from nth order polynomial.
    """
    value = np.zeros(order + 1)
    # Matrix for computation
    compute_mat = np.eye(order + 1)
    for i in range(0, order + 1):
        value[i] = np.polyval(np.polyder(compute_mat[i], k), t)
    return value

if __name__ == "__main__":
    time = [1, 1]
    position = np.array([[0,0,0,0],[1,1,1,1], [2,2,2,2]])
    velocity = 0
    #velocity = np.array([[0,0,0,0],[0,0,0,0], [0,0,0,0]])
    acceleration = np.array([[0,0,0,0],[0,0,0,0], [0,0,0,0]])
    get_solution(time,position, velocity, acceleration)
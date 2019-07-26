import numpy as np
from cvxopt import matrix, spdiag


# TODO : consider that cost matrix and mapping matrix are constant over all dimensions.
# TODO : consider easy method to construct mapping matrix
class UnConstraintQpMatrix:
    """
     Class for computing matrix which is used at unconstrained QP.
    Especially compute A matrix.
    Every fixed or free constraints will be considered.
    s.t. { A*c = b  }

     To generate global trajectory which has many way points, unconstrained optimization is used to avoid singular.
    I assume drone start and arrive at rest.
    """
    def __init__(self, m, keyframe, k_r, time_scaling):
        self.order = 9
        self.m = m
        self.keyframe = keyframe

        # k_r = 4
        self.k_r = k_r
        self.time_scaling = time_scaling

        # dimension of variable.
        # 3: [ x y z ]     control psi independently
        # 4: [ x y z psi ]
        self.n = 3

        # polynomial coefficient
        self.poly_coef = np.zeros((self.k_r + 1, 2, self.order + 1))
        # This is a helper variable to get the coefficient for n-th order polynomial with k-th derivative at time t.
        for t in range(0, 2):
            for i in range(0, self.k_r + 1):
                compute_mat = np.eye(self.order + 1)
                values = np.zeros(self.order + 1)
                for j in range(0, self.order + 1):
                    tempCoeffs = compute_mat[j, :]
                    for k in range(0, i):
                        tempCoeffs = np.polyder(tempCoeffs)
                    values[j] = np.polyval(tempCoeffs, t)
                self.poly_coef[i][t] = values

    def compute_p(self, mu_r):
        """
         min { 1/2 c.T * P * c + q * c }
        Build P matrix
        """
        p = []
        polynomial_r = np.ones(self.order + 1)
        for i in range(0, self.k_r):
            polynomial_r = np.polyder(polynomial_r)

        for i in range(0, self.m):
            p_x = np.zeros((self.order + 1, self.order + 1))
            p_y = np.zeros((self.order + 1, self.order + 1))
            p_z = np.zeros((self.order + 1, self.order + 1))
            for j in range(0, self.order + 1):
                for k in range(j, self.order + 1):
                    # position
                    if j <= len(polynomial_r) - 1 and k <= len(polynomial_r) - 1:
                        order_t_r = ((self.order - self.k_r - j) + (self.order - self.k_r - k))
                        if j == k:
                            p_x[j, k] = np.power(polynomial_r[j], 2) / (order_t_r + 1)
                            p_y[j, k] = np.power(polynomial_r[j], 2) / (order_t_r + 1)
                            p_z[j, k] = np.power(polynomial_r[j], 2) / (order_t_r + 1)

                        else:
                            p_x[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1)
                            p_y[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1)
                            p_z[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1)

            p_x = matrix(p_x) * mu_r
            p_y = matrix(p_y) * mu_r
            p_z = matrix(p_z) * mu_r
            if i == 0:
                p = spdiag([p_x, p_y, p_z])
            else:
                p = spdiag([p, p_x, p_y, p_z])

        p = matrix(p)
        p = p + matrix(np.transpose(p))
        p = p * 0.5
        return p

    def waypoint_constraint(self):
        """
         way point constraint
        In each segment, start and end of polynomial must satisfy way point constraint
        Drone need to pass these points
        """

        A1 = np.zeros((2 * self.m * self.n, self.n * (self.order + 1) * self.m))
        b1 = np.ones(2 * self.m * self.n)

        for i in range(0, self.m):
            way_point = self.keyframe[:, i]

            if i == 0:
                # Initial position: First polynomial which has time as 0.
                values = self.poly_coef[0, 0]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                            self.order + 1) + self.order + 1] = values
                    A1[k, :] = a
                b1[0: self.n] = way_point

                # Final position: Last polynomial which has time as 1.
                values = self.poly_coef[0, 1]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[(self.m - 1) * (self.order + 1) * self.n + k * (self.order + 1): (self.m - 1) * (
                            self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                    A1[k + self.n, :] = a
                b1[self.n: 2 * self.n] = self.keyframe[:, self.m]

            else:
                # Elsewhere: polynomial of each segment which has time as 0, 1 except initial, final point.
                values = self.poly_coef[0, 1]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[(i - 1) * (self.order + 1) * self.n + k * (self.order + 1): (i - 1) * (
                            self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                    A1[k + 2 * self.n * i, :] = a
                b1[(2 * self.n * i): (2 * self.n * i) + self.n] = way_point

                values = self.poly_coef[0, 0]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * self.m)
                    a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                            self.order + 1) + self.order + 1] = values
                    A1[k + 2 * self.n * i + self.n, :] = a
                b1[(2 * self.n * i) + self.n: (2 * self.n * i) + 2 * self.n] = way_point

        A1 = matrix(A1)
        b1 = matrix(b1)
        return A1, b1

    def derivative_constraint(self):
        """
         Derivative constraint
        In each segment, start and end of polynomial must satisfy derivative constraint
        This is very important for continuity of trajectory

         Also, Every fixed or free constraints will be considered in unconstrained QP.
        I assume drone start and arrive at rest.

        Construct re-ordering matrix.
        dF : fixed ,dP: free constraints.
        [ dF dP ]
        fixed constraint : 2 * m + 4 * ( m + 1) = 6 m + 4  --> A2
        free constraint : 10 * m - ( 6 m + 4 ) = 4 m - 4   --> A3
        """
        # Derivative Constraint
        # It should guarantee continuity until snap.
        # In differential flatness model, states are functions of jerk and snap.

        # For position x y z: yaw excluded here
        # Sometimes, yaw always 0.
        A2 = np.zeros(((4 * self.m + 4) * self.n, self.n * (self.order + 1) * self.m))
        b2 = np.ones(((4 * self.m + 4) * self.n, 1))
        A3 = np.zeros(((4 * self.m - 4) * self.n, self.n * (self.order + 1) * self.m))
        b3 = np.ones(((4 * self.m - 4) * self.n, 1))

        for i in range(0, self.m):
            for h in range(0, self.k_r):
                if i == 0:
                    # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                    values = self.poly_coef[h + 1, 0]

                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                                    self.order + 1) + self.order + 1] = values
                        A2[k + h * self.n, :] = a
                        b2[k + h * self.n] = 0

                    # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                    values = self.poly_coef[h + 1, 1]

                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        a[(self.m - 1) * (self.order + 1) * self.n + k * (self.order + 1): (self.m - 1) * (
                                    self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                        A2[k + h * self.n + self.n * self.k_r, :] = a
                        b2[k + h * self.n + self.n * self.k_r] = 0

                else:
                    # Elsewhere: polynomial of each segment which has time as 0, 1 except initial, final point.
                    end_values = self.poly_coef[h + 1, 1]
                    start_values = self.poly_coef[h + 1, 0]

                    # Fixed/specified constraints
                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        a[(i - 1) * (self.order + 1) * self.n + k * (self.order + 1): (i - 1) * (
                                    self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = end_values
                        a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                                    self.order + 1) + self.order + 1] = -start_values
                        A2[k + h * self.n + (i + 1) * self.n * self.k_r, :] = a
                        b2[k + h * self.n + (i + 1) * self.n * self.k_r] = 0

                    # Free/unspecified constraints
                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        a[(i - 1) * (self.order + 1) * self.n + k * (self.order + 1): (i - 1) * (
                                self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = end_values
                        A3[k + h * self.n + (i - 1) * self.n * self.k_r, :] = a
                        b3[k + h * self.n + (i - 1) * self.n * self.k_r, :] = 0

        A2 = matrix(A2)
        A3 = matrix(A3)
        b2 = matrix(b2)
        b3 = matrix(b3)
        A2 = matrix([A2, A3])
        b2 = matrix([b2, b3])
        return A2, b2

    def compute_p_1axis(self):
        """
         min { 1/2 c.T * P * c + q * c }
        Build P matrix in 1 axis

        In unconstrained QP, P matrix have same elements over all dimensions
        because it depends on time only. Moreover, if I use time scaling method, P matrix have same elements over all
        segments
        """
        P = []
        polynomial_r = np.ones(self.order + 1)
        for i in range(0, self.k_r):
            polynomial_r = np.polyder(polynomial_r)

        p = np.zeros((self.order + 1, self.order + 1))
        for j in range(0, self.order + 1):
            for k in range(j, self.order + 1):
                # position
                if j <= len(polynomial_r) - 1 and k <= len(polynomial_r) - 1:
                    order_t_r = ((self.order - self.k_r - j) + (self.order - self.k_r - k))
                    if j == k:
                        p[j, k] = np.power(polynomial_r[j], 2) / (order_t_r + 1)
                    else:
                        p[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1)

        p = matrix(p)
        for i in range(self.m):
            if i == 0:
                P = spdiag([p])
            else:
                P = spdiag([P, p])

        P = matrix(P)
        P = P + matrix(np.transpose(P))
        P = P * 0.5
        return P

    def compute_A_1axis(self):
        # A is square matrix
        A = np.zeros((2 * self.m * (self.k_r + 1), (self.order + 1) * self.m))

        for i in range(0, self.m):
            for h in range(0, self.k_r + 1):
                # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                start_values = self.poly_coef[h, 0]

                # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                end_values = self.poly_coef[h, 1]

                if i == 0:
                    A[2 * (self.k_r + 1) * i + h, i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values
                    A[2 * (self.k_r + 1) * i + h + (self.k_r + 1), i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
                else:
                    if h == 0:
                        A[2 * (self.k_r + 1) * i + h, i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values
                        A[2 * (self.k_r + 1) * i + h + (self.k_r + 1),
                        i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
                    else:
                        A[2 * (self.k_r + 1) * i + h,
                        i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values
                        A[2 * (self.k_r + 1) * i + h,
                        (i - 1) * (self.order + 1): (i - 1) * (self.order + 1) + (self.order + 1)] = -end_values
                        A[2 * (self.k_r + 1) * i + h + (self.k_r + 1),
                        i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
        return A

    def end_derivative(self):
        """
         Function to construct end constraints at each segments.
        """
        b = np.zeros((3, 2 * self.m * (self.k_r + 1)))
        for i in range(3):
            for j in range(self.m):
                b[i][2 * (self.k_r + 1) * j] = self.keyframe[i][j]
                b[i][2 * (self.k_r + 1) * j + 5] = self.keyframe[i][j + 1]

        return b

    def fixed_free_constraints(self, b):
        """
         Function to re order end derivative constraints to fixed and free constraints
        """
        bF = [[], [], []]
        bP = [[], [], []]
        for i in range(3):
            for j in range(self.m):
                # start point of segment
                for k in range(self.k_r + 1):
                    if j == self.m - 1:
                            bF[i].append(b[i][2 * (self.m - 1) * (self.k_r + 1) + k])
                    else:
                        if k == 0:
                            bF[i].append(b[i][2 * j * (self.k_r + 1)])
                        else:
                            bF[i].append(b[i][2 * j * (self.k_r + 1) + k])

                # end point of segment
                for k in range(self.k_r + 1):
                    if j == self.m - 1:
                            bF[i].append(b[i][2 * (self.m - 1) * (self.k_r + 1) + (self.k_r + 1) + k])
                    else:
                        if k == 0:
                            bF[i].append(b[i][2 * j * (self.k_r + 1) + (self.k_r + 1) + k])
                        else:
                            bP[i].append(b[i][2 * j * (self.k_r + 1) + (self.k_r + 1) + k])

        return [bF, bP]

    def mapping_matrix(self):
        """
         Function to construct mapping matrix
        """
        M = np.zeros((2 * self.m * (self.k_r + 1), 2 * self.m * (self.k_r + 1)), dtype=int)
        f = 0
        p = 0
        for j in range(self.m):
            for k in range(self.k_r + 1):
                if j == self.m - 1:
                    M[2 * j * (self.k_r + 1) + k][f] = 1
                else:
                    if k == 0:
                        M[2 * j * (self.k_r + 1) + k][f] = 1
                    else:
                        M[2 * j * (self.k_r + 1) + k][f] = 1
                f = f + 1

            for k in range(self.k_r + 1):
                if j == self.m - 1:
                    M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][f] = 1
                else:
                    if k == 0:
                        M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][f] = 1
                    else:
                        M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][6 * self.m + 4 + p] = 1
                        f = f - 1
                        p = p + 1
                f = f + 1

        return M

    def compute_p_1axis_replanning(self, m):
        """
         min { 1/2 c.T * P * c + q * c }
        Build P matrix in 1 axis

        In unconstrained QP, P matrix have same elements over all dimensions
        because it depends on time only. Moreover, if I use time scaling method, P matrix have same elements over all
        segments
        """
        P = []
        polynomial_r = np.ones(self.order + 1)
        for i in range(0, self.k_r):
            polynomial_r = np.polyder(polynomial_r)

        p = np.zeros((self.order + 1, self.order + 1))
        for j in range(0, self.order + 1):
            for k in range(j, self.order + 1):
                # position
                if j <= len(polynomial_r) - 1 and k <= len(polynomial_r) - 1:
                    order_t_r = ((self.order - self.k_r - j) + (self.order - self.k_r - k))
                    if j == k:
                        p[j, k] = np.power(polynomial_r[j], 2) / (order_t_r + 1)
                    else:
                        p[j, k] = 2 * polynomial_r[j] * polynomial_r[k] / (order_t_r + 1)

        p = matrix(p)
        for i in range(m):
            if i == 0:
                P = spdiag([p])
            else:
                P = spdiag([P, p])

        P = matrix(P)
        P = P + matrix(np.transpose(P))
        P = P * 0.5
        return P

    def compute_A_1axis_replanning(self, m=5):
        """
         Function to compute A matrix when re-planning.
        It has less fixed constraints because we only know start and goal point.

        segment : m
        Fixed constraints: 5 * 2 +  5 * (m - 1) = 5m + 5
        Free constraints: 10m - (5m +5) = 5m - 5
        """
        A = np.zeros((2 * m * (self.k_r + 1), (self.order + 1) * m))

        for i in range(0, m):
            for h in range(0, self.k_r + 1):
                # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                start_values = self.poly_coef[h, 0]

                # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                end_values = self.poly_coef[h, 1]

                if i == 0:
                    A[2 * (self.k_r + 1) * i + h, i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values
                    A[2 * (self.k_r + 1) * i + h + (self.k_r + 1), i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
                else:

                    A[2 * (self.k_r + 1) * i + h,
                    i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values
                    A[2 * (self.k_r + 1) * i + h,
                    (i - 1) * (self.order + 1): (i - 1) * (self.order + 1) + (self.order + 1)] = -end_values
                    A[2 * (self.k_r + 1) * i + h + (self.k_r + 1),
                    i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
        return A

    def end_derivative_replanning(self, m, start, goal):
        """
         Function to construct end constraints at each segments when re-planning.
        For now, We know start state and goal state.
        """
        b = np.zeros((3, 2 * m * (self.k_r + 1)))
        for i in range(3):
            b[i][:self.k_r + 1] = start[i]
            b[i][2 * (self.k_r + 1) * (m - 1) + (self.k_r + 1):] = goal[i]

        return b

    def fixed_free_constraints_replanning(self, b, m=5):
        """
         Function to re order end derivative constraints to fixed and free constraints
        """
        bF = [[], [], []]
        bP = [[], [], []]
        for i in range(3):
            for j in range(m):
                # start point of segment
                for k in range(self.k_r + 1):
                    bF[i].append(b[i][2 * j * (self.k_r + 1) + k])

                # end point of segment
                for k in range(self.k_r + 1):
                    if j == m - 1:
                        bF[i].append(b[i][2 * (m - 1) * (self.k_r + 1) + (self.k_r + 1) + k])
                    else:
                        bP[i].append(b[i][2 * j * (self.k_r + 1) + (self.k_r + 1) + k])

        return [bF, bP]

    def mapping_matrix_replanning(self, m):
        """
         Function to construct mapping matrix
        """
        M = np.zeros((2 * m * (self.k_r + 1), 2 * m * (self.k_r + 1)), dtype=int)
        f = 0
        p = 0
        for j in range(m):
            for k in range(self.k_r + 1):
                M[2 * j * (self.k_r + 1) + k][f] = 1
                f = f + 1

            for k in range(self.k_r + 1):
                if j == m - 1:
                    M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][f] = 1
                else:
                    M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][5 * m + 5 + p] = 1
                    f = f - 1
                    p = p + 1
                f = f + 1

        return M


if __name__ == "__main__":
    waypoint = np.array([[0, 0, 0], [1, 5, 5], [20, 20, 20], [30, 30, 30]])
    m = len(waypoint) - 1
    time = np.ones(m)
    waypoint = np.transpose(waypoint)
    unqp = UnConstraintQpMatrix(m, waypoint, 4, time)

    P = unqp.compute_p_1axis()
    np.savetxt("P.csv", P, delimiter=",")

    A = unqp.compute_A_1axis()
    np.savetxt("A2.csv", A, delimiter=",")

    b = unqp.end_derivative()
    [bF, bP] = unqp.fixed_free_constraints(b)

    M = unqp.mapping_matrix()
    np.savetxt("M.csv", M, delimiter=",")

    A = matrix(A)
    M = matrix(M)

    invA = matrix(np.linalg.inv(A))
    R = M.T * invA.T * P * invA * M

    RFP = R[0:(6 * m + 4), (6 * m + 4):]
    RPP = R[(6 * m + 4):, (6 * m + 4):]

    invRPP = matrix(np.linalg.inv(RPP))
    X = -invRPP * RFP.T

    cost = 0
    solution = np.zeros((3, 10 * m))
    for i in range(3):
        dF = np.array(bF[i])
        dF = matrix(dF)
        dP = X * matrix(dF)

        d = matrix([dF, dP])
        p = invA * M * d

        solution[i][:] = np.array(p.T)
        # value of cost function which is minimized by optimization
        cost_val = p.T * P * p
        cost = cost + cost_val

    #print cost


    P = unqp.compute_p_1axis_replanning(5)
    A = unqp.compute_A_1axis_replanning(5)
    np.savetxt("A.csv", A, delimiter=",")
    m = 5
    start = [[1, 1, 1, 1, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    goal = [[2, 2, 2, 2, 2], [1, 1, 1, 1, 1], [2, 2, 2, 2, 2]]
    b = unqp.end_derivative_replanning(5, start, goal)
    [bF, bP] = unqp.fixed_free_constraints_replanning(b, 5)

    M = unqp.mapping_matrix_replanning(5)
    M = matrix(M)
    np.savetxt("M.csv", M, delimiter=",")
    bFP = np.hstack((bF[0], bP[0]))

    invA = matrix(np.linalg.inv(A))
    R = M.T * invA.T * P * invA * M

    RFP = R[0:(5 * m + 5), (5 * m + 5):]
    RPP = R[(5 * m + 5):, (5 * m + 5):]

    invRPP = matrix(np.linalg.inv(RPP))
    X = -invRPP * RFP.T
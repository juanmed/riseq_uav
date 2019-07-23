import numpy as np
from cvxopt import matrix, spdiag


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
        fixed constraint : 2 * m + 4 * ( m + 1) = 6 m + 4
        free constraint : 10 * m - ( 6 m + 4 ) = 4 m - 4
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

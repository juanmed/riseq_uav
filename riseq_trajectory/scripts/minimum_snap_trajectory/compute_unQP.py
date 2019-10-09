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
from scipy.linalg import block_diag


class UnConstraintQpMatrix:
    """
     Class for computing matrix which is used at unconstrained QP.

     Traditional formulation becomes ill-conditioned for more than several segments,
    polynomials of high order, and when widely varying segment times are involved.
    It is only useful for short trajectory.
    In other words, inversion of matrices of that may be very close to singular.
    This reformulation is substantially more stable than the method above, allowing the joint optimization of more
    than 50 polynomial segments in a single matrix operation without encountering numerical issues[1].

     We briefly summarize the essentials in order to understand the non-linear solution and point out optimizations
    for both numerical stability and to save computation time.

     The inverse of matrix can be evaluated if the matrix is square. If not, psuedo inverse should be calculated[2].
    To make square matrix, order of polynomial needs to be 9 so that have 10 coefficients because we have 10 constraints
    2 position, 2 velocity, 2 acceleration. 2 jerk, 2 snap.
     Next thing is determine fixed/specified constraints, and free/unspecified constraints. position should be fixed
    constraints. In derivative constraints, initial and final value is set as fixed constraints. Also for continuity
    of end of the ith segment and beginning of the (i+1)th segment, set fixed constraints. Else things is free
    constraints.

     Note that both the cost-Matrix Q and mapping matrix A only depend on the segment time Ts,i and thus are constant
    over all dimensions for the segment, which allow for computation-time savings in the case of multiple dimensions.

    qp(P, q, None, None, A, b)
    min J = { 1/2 * c.T * P * c } ---------- c is array of polynomial of flat output [ x y z psi ].
    s.t. { A*c = b  }

    For formulating unconstrained_qp, it can't have inequality constraint.
    c = inv(A)*b
    min J = { 1/2 * b.T * inv(A).T * P * inv(A) * b }

    J = b.T * inv(A).T * P * inv(A) * b
    J = d.T * R * d    -------------------- d = [ dF dP ] R = [[RFF RFP],  F : fixed/specified derivatives
                                                                [RPF RPP]]  P : free/unspecified derivatives

     Differentiating J and equating to zero yields the vector of optimal values for the free derivatives in terms of the
    fixed/specified derivatives and the ost matrix [1]

    dP* = -inv(RPP)*RFP.T*dF
    """
    def __init__(self, order, k_r):
        # Usually order is 9 for square matrix, 10 coefficients 10 constraints
        self.order = order

        # k_r = 4
        self.k_r = k_r

        # dimension of variable.
        # 3: [ x y z ]     control psi independently
        # 4: [ x y z psi ]
        self.n = 3

        # polynomial coefficient
        # k th derivative with time 0, 1 with n th order
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

    def compute_P(self, m, time):
        """
         min { 1/2 c.T * P * c + q * c }
        Build P matrix

        Compute x y z at once.
        """
        P = []
        polynomial_r = np.ones(self.order + 1)
        for i in range(0, self.k_r):
            polynomial_r = np.polyder(polynomial_r)

        for i in range(0, m):
            p_x = np.zeros((self.order + 1, self.order + 1))
            p_y = np.zeros((self.order + 1, self.order + 1))
            p_z = np.zeros((self.order + 1, self.order + 1))
            for j in range(0, self.order + 1):
                for k in range(j, self.order + 1):
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

            if i == 0:
                P = block_diag(p_x, p_y, p_z)  # / time[i]**7
            else:
                P = block_diag(P, p_x, p_y, p_z)  # / time[i]**7

        P = P + np.transpose(P)
        P = P * 0.5
        return P

    def waypoint_constraint(self, waypoint, m):
        """
         way point constraint
        In each segment, start and end of polynomial must satisfy way point constraint
        Drone need to pass these points
        """
        A1 = np.zeros((2 * m * self.n, self.n * (self.order + 1) * m))
        b1 = np.ones(2 * m * self.n)

        for i in range(0, m):

            if i == 0:
                # Initial position: First polynomial which has time as 0.
                values = self.poly_coef[0, 0]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * m)
                    a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                            self.order + 1) + self.order + 1] = values
                    A1[k, :] = a
                b1[0: self.n] = waypoint[:, 0]

                # Final position: Last polynomial which has time as 1.
                values = self.poly_coef[0, 1]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * m)
                    a[(m - 1) * (self.order + 1) * self.n + k * (self.order + 1): (m - 1) * (
                            self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                    A1[k + self.n, :] = a
                b1[self.n: 2 * self.n] = waypoint[:, m]

            else:
                # Elsewhere: polynomial of each segment which has time as 0, 1 except initial, final point.
                values = self.poly_coef[0, 1]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * m)
                    a[(i - 1) * (self.order + 1) * self.n + k * (self.order + 1): (i - 1) * (
                            self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                    A1[k + 2 * self.n * i, :] = a
                b1[(2 * self.n * i): (2 * self.n * i) + self.n] = waypoint[:, i]

                values = self.poly_coef[0, 0]
                for k in range(0, self.n):
                    a = np.zeros(self.n * (self.order + 1) * m)
                    a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                            self.order + 1) + self.order + 1] = values
                    A1[k + 2 * self.n * i + self.n, :] = a
                b1[(2 * self.n * i) + self.n: (2 * self.n * i) + 2 * self.n] = waypoint[:, i]

        return A1, b1

    def derivative_constraint(self, m, time):
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
        A2 = np.zeros(((4 * m + 4) * self.n, self.n * (self.order + 1) * m))
        b2 = np.ones(((4 * m + 4) * self.n, 1))
        A3 = np.zeros(((4 * m - 4) * self.n, self.n * (self.order + 1) * m))
        b3 = np.ones(((4 * m - 4) * self.n, 1))

        for i in range(0, m):
            for h in range(0, self.k_r):
                if i == 0:
                    # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                    # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                    end_values = self.poly_coef[h + 1, 1] / (time[m-1] ** (h + 1))
                    start_values = self.poly_coef[h + 1, 0] / (time[0] ** (h + 1))

                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * m)
                        a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                                    self.order + 1) + self.order + 1] = start_values
                        A2[k + h * self.n, :] = a
                        b2[k + h * self.n] = 0

                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * m)
                        a[(m - 1) * (self.order + 1) * self.n + k * (self.order + 1): (m - 1) * (
                                    self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = end_values
                        A2[k + h * self.n + self.n * self.k_r, :] = a
                        b2[k + h * self.n + self.n * self.k_r] = 0

                else:
                    # Elsewhere: polynomial of each segment which has time as 0, 1 except initial, final point.
                    end_values = self.poly_coef[h + 1, 1] / (time[i-1] ** (h + 1))
                    start_values = self.poly_coef[h + 1, 0] / (time[i] ** (h + 1))

                    # Fixed/specified constraints
                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * m)
                        a[(i - 1) * (self.order + 1) * self.n + k * (self.order + 1): (i - 1) * (
                                    self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = end_values
                        a[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                                    self.order + 1) + self.order + 1] = -start_values
                        A2[k + h * self.n + (i + 1) * self.n * self.k_r, :] = a
                        b2[k + h * self.n + (i + 1) * self.n * self.k_r] = 0

                    # Free/unspecified constraints
                    for k in range(0, self.n):
                        a = np.zeros(self.n * (self.order + 1) * m)
                        a[(i - 1) * (self.order + 1) * self.n + k * (self.order + 1): (i - 1) * (
                                self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = end_values
                        A3[k + h * self.n + (i - 1) * self.n * self.k_r, :] = a
                        b3[k + h * self.n + (i - 1) * self.n * self.k_r, :] = 0

        A2 = np.vstack((A2, A3))
        b2 = np.vstack((b2, b3))
        return A2, b2

    def compute_P_1axis(self, m, time):
        """
         min { 1/2 c.T * P * c + q * c }
        Build P matrix in 1 axis

        In unconstrained QP, P matrix have same elements over all dimensions
        because it depends on time only. Moreover, if I use time scaling method, P matrix have same elements over all
        segments

        Compute just 1 dimension.
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

        for i in range(m):
            if i == 0:
                P = block_diag(p/time[i]**7)
            else:
                P = block_diag(P, p/time[i]**7)

        P = P + np.transpose(P)
        P = P * 0.5
        return P

    def compute_A_global(self, m, time):
        """
        Compute A matrix only about 1 dimension for local trajectory
        """
        A = np.zeros((2 * m * (self.k_r + 1), (self.order + 1) * m))

        for i in range(0, m):
            for h in range(0, self.k_r + 1):
                if i == 0:
                    # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                    start_values = self.poly_coef[h, 0] / (time[0]**h)

                    # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                    end_values = self.poly_coef[h, 1] / (time[0]**h)

                    A[2 * (self.k_r + 1) * i + h, i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values
                    A[2 * (self.k_r + 1) * i + h + (self.k_r + 1), i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values

                else:
                    if h == 0:
                        # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                        start_values = self.poly_coef[h, 0] / (time[i]**h)

                        # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                        end_values = self.poly_coef[h, 1] / (time[i - 1]**h)

                        A[2 * (self.k_r + 1) * i + h,
                        i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values

                        # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                        end_values = self.poly_coef[h, 1] / (time[i]**h)

                        A[2 * (self.k_r + 1) * i + h + (self.k_r + 1),
                        i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values

                    else:
                        # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                        start_values = self.poly_coef[h, 0] / (time[i]**h)

                        # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                        end_values = self.poly_coef[h, 1] / (time[i - 1]**h)
                        A[2 * (self.k_r + 1) * i + h,
                        i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = -start_values
                        A[2 * (self.k_r + 1) * i + h,
                        (i - 1) * (self.order + 1): (i - 1) * (self.order + 1) + (self.order + 1)] = end_values

                        # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                        end_values = self.poly_coef[h, 1] / (time[i]**h)

                        A[2 * (self.k_r + 1) * i + h + (self.k_r + 1),
                        i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
        return A

    def compute_A_local(self, m, time):
        """
        Compute A matrix only about 1 dimension for local trajectory
        """
        A = np.zeros((2 * m * (self.k_r + 1), (self.order + 1) * m))

        for i in range(0, m):
            for h in range(0, self.k_r + 1):
                if i == 0:
                    # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                    start_values = self.poly_coef[h, 0] / (time[0]**h)

                    # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                    end_values = self.poly_coef[h, 1] / (time[0]**h)

                    A[2 * (self.k_r + 1) * i + h, i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = start_values
                    A[2 * (self.k_r + 1) * i + h + (self.k_r + 1), i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
                else:
                    # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                    start_values = self.poly_coef[h, 0] / (time[i]**h)

                    # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                    end_values = self.poly_coef[h, 1] / (time[i - 1]**h)

                    A[2 * (self.k_r + 1) * i + h,
                    i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = -start_values
                    A[2 * (self.k_r + 1) * i + h,
                    (i - 1) * (self.order + 1): (i - 1) * (self.order + 1) + (self.order + 1)] = end_values

                    # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                    end_values = self.poly_coef[h, 1] / (time[i] **h)

                    A[2 * (self.k_r + 1) * i + h + (self.k_r + 1),
                    i * (self.order + 1): i * (self.order + 1) + (self.order + 1)] = end_values
        return A

    def end_derivative_global(self, waypoint, m, start, goal):
        """
         Function to construct end constraints at each segments for global trajectory.

        Assume that all way points is already determined.
        Free constraints are derivative term at points.

        We know start state and goal state.
        In almost case, drone starts and arrives at rest(vel=0 acc=0).

        segment : m
        fixed constraint : 2 * m + 4 * ( m + 1) = 6 m + 4
        free constraint : 10 * m - ( 6 m + 4 ) = 4 m - 4
        """
        b = np.zeros((3, 2 * m * (self.k_r + 1)))
        for i in range(3):
            for j in range(m):
                b[i][2 * (self.k_r + 1) * j] = waypoint[i][j]
                b[i][2 * (self.k_r + 1) * j + 5] = waypoint[i][j + 1]

        for i in range(3):
            b[i][:self.k_r + 1] = start[i]
            b[i][2 * (self.k_r + 1) * (m - 1) + (self.k_r + 1):] = goal[i]

        return b

    def end_derivative_local(self, m, start):
        """
         Function to construct end constraints at each segments for local trajectory.

        Only Initial way point is determined.
        Other way points are free constraints.
        We know start state. It is from last trajectory.

        segment : m
        fixed constraint : 10 m - 5 m = 5m
        free constraint : 10 m - 5 m = 5m
        """
        b = np.zeros((3, 2 * m * (self.k_r + 1)))

        for i in range(3):
            b[i][:self.k_r + 1] = start[i]

        return b

    def fixed_free_constraints_global(self, b, m):
        """
         Function to re order end derivative constraints to fixed and free constraints for global trajectory.

        Assume that all way points is already determined.
        Free constraints are derivative term at points.

        segment : m
        fixed constraint : 2 * m + 4 * ( m + 1) = 6 m + 4
        free constraint : 10 * m - ( 6 m + 4 ) = 4 m - 4
        """
        bF = [[], [], []]
        bP = [[], [], []]
        for i in range(3):
            for j in range(m):

                # start point of segment
                for k in range(self.k_r + 1):
                    if j == 0:
                        bF[i].append(b[i][k])
                    else:
                        # Fixed Constraints for continuity
                        bF[i].append(b[i][2 * j * (self.k_r + 1) + k])

                # end point of segment
                for k in range(self.k_r + 1):
                    if j == m - 1:
                            bF[i].append(b[i][2 * (m - 1) * (self.k_r + 1) + (self.k_r + 1) + k])
                    else:
                        # Fixed Constraints for each way point
                        if k == 0:
                            bF[i].append(b[i][2 * j * (self.k_r + 1) + (self.k_r + 1) + k])

                        # Free Constraints for end derivative
                        else:
                            bP[i].append(b[i][2 * j * (self.k_r + 1) + (self.k_r + 1) + k])

        return [bF, bP]

    def fixed_free_constraints_local(self, b, m):
        """
         Function to re order end derivative constraints to fixed and free constraints for local trajectory.

        Only Initial way point is determined. It is from last trajectory.
        Other way points are free constraints
        We know start state. It is from last trajectory.

        segment : m
        fixed constraint : 10 m - 5 m = 5m
        free constraint : 10 m - 5 m = 5m
        """
        bF = [[], [], []]
        bP = [[], [], []]
        for i in range(3):
            for j in range(m):

                # start point of segment
                for k in range(self.k_r + 1):
                    # Fixed Constraints for continuity
                    bF[i].append(b[i][2 * j * (self.k_r + 1) + k])

                # end point of segment
                for k in range(self.k_r + 1):
                    # Free Constraints for way point and end derivative
                    bP[i].append(b[i][2 * j * (self.k_r + 1) + (self.k_r + 1) + k])

        return [bF, bP]

    def mapping_matrix_global(self, m):
        """
         Function to construct mapping matrix for global trajectory

        segment : m
        fixed constraint : 2 * m + 4 * ( m + 1) = 6 m + 4
        free constraint : 10 * m - ( 6 m + 4 ) = 4 m - 4
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
                    if k == 0:
                        M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][f] = 1
                    else:
                        M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][6 * m + 4 + p] = 1
                        f = f - 1
                        p = p + 1
                f = f + 1

        return M

    def mapping_matrix_local(self, m):
        """
         Function to construct mapping matrix for local trajectory

        segment : m
        fixed constraint : 10 m - 5 m = 5m
        free constraint : 10 m - 5 m = 5m
        """
        M = np.zeros((2 * m * (self.k_r + 1), 2 * m * (self.k_r + 1)), dtype=int)
        f = 0
        p = 0
        for j in range(m):
            for k in range(self.k_r + 1):
                M[2 * j * (self.k_r + 1) + k][f] = 1
                f = f + 1

            for k in range(self.k_r + 1):
                M[2 * j * (self.k_r + 1) + (self.k_r + 1) + k][5 * m + p] = 1
                p = p + 1

        return M


def main():
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


if __name__ == "__main__":
    main1()
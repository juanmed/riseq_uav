import numpy as np
from cvxopt import matrix, spdiag


class QpMatrix:
    """
    Contain every function for computing matrix which is used at quadratic programming
    """
    def __init__(self, order, m, keyframe, k_r, k_psi, time_scaling):
        self.order = order
        self.m = m
        self.keyframe = keyframe
        self.k_r = k_r
        self.k_psi = k_psi
        self.time_scaling = time_scaling
        self.n = 4

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

    def compute_p(self, mu_r, mu_psi):
        """
         min { 1/2 c.T * P * c + q * c }
        Build P matrix
        """
        p = []
        polynomial_r = np.ones(self.order + 1)
        for i in range(0, self.k_r):
            polynomial_r = np.polyder(polynomial_r)

        polynomial_psi = np.ones(self.order + 1)
        for i in range(0, self.k_psi):
            polynomial_psi = np.polyder(polynomial_psi)

        for i in range(0, self.m):
            p_x = np.zeros((self.order + 1, self.order + 1))
            p_y = np.zeros((self.order + 1, self.order + 1))
            p_z = np.zeros((self.order + 1, self.order + 1))
            p_psi = np.zeros((self.order + 1, self.order + 1))
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

                    # yaw
                    if j <= len(polynomial_psi) - 1 and k <= len(polynomial_psi) - 1:
                        order_t_psi = ((self.order - self.k_psi - j) + (self.order - self.k_psi - k))
                        if j == k:
                            p_psi[j, k] = np.power(polynomial_psi[j], 2) / (order_t_psi + 1)
                        else:
                            p_psi[j, k] = 2 * polynomial_psi[j] * polynomial_psi[k] / (order_t_psi + 1)
            p_x = matrix(p_x) * mu_r
            p_y = matrix(p_y) * mu_r
            p_z = matrix(p_z) * mu_r
            p_psi = matrix(p_psi) * mu_psi
            if i == 0:
                p = spdiag([p_x, p_y, p_z, p_psi])
            else:
                p = spdiag([p, p_x, p_y, p_z, p_psi])

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

    def derivative_constraint(self, current_state):
        """
         derivative constraint
        In each segment, start and end of polynomial must satisfy derivative constraint
        This is very important for continuity of trajectory
        """
        # constraint_data_r[m, k_r]
        # 0   -> False ... In this case, Initial, Final velocity and acceleration is zero.
        # 1   -> True  ... continuity, in way point it must be continuous.
        constraint_data_r = np.zeros((self.m, self.k_r))
        if self.k_r >= 1:
            constraint_data_r[0, 0] = 0
            constraint_data_r[1:self.m, 0] = 1
        if self.k_r >= 2:
            constraint_data_r[0, 1] = 0
            constraint_data_r[1:self.m, 1] = 1
        if self.k_r >= 3:
            constraint_data_r[0, 2] = 0
            constraint_data_r[1:self.m, 2] = 1
        if self.k_r >= 4:
            constraint_data_r[0, 3] = 0
            constraint_data_r[1:self.m, 3] = 1

        # Derivative Constraint
        # It should guarantee continuity until snap.
        # In differential flatness model, states are functions of jerk and snap.

        # For position x y z: yaw excluded here (n-1)
        # Sometimes, yaw always 0.
        A2 = np.zeros((2 * self.m * (self.n - 1) * self.k_r, self.n * (self.order + 1) * self.m))
        b2 = np.ones((2 * self.m * (self.n - 1) * self.k_r, 1)) * 0.001

        for i in range(0, self.m):
            for h in range(0, self.k_r):
                if i == 0:
                    # Initial vel, acc, jerk, snap: First polynomial which has time as 0.
                    values = self.poly_coef[h+1, 0]

                    continuity = False
                    if constraint_data_r[i, h] == 1:
                        # Continuity
                        continuity = True

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is True:
                            a[i*(self.order+1)*self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                            a[(self.m-1) * (self.order + 1) * self.n + k * (self.order+1):  (self.m -1) * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = -values
                            A2[k + h*(self.n-1), :] = a
                            b2[k + h*(self.n-1)] = 0
                        else:
                            a[i*(self.order+1)*self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = values
                            A2[k + h*(self.n-1), :] = a
                            b2[k + h*(self.n-1)] = current_state[h+1][k] * np.power(self.time_scaling[0], h + 1)        # constraint_data_r[i, h]

                    # Final vel, acc, jerk, snap: Last polynomial which has time as 1.
                    values = self.poly_coef[h+1, 1]

                    continuity = False
                    if constraint_data_r[i, h] == 1:
                        continuity = True
                        # Continuity

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is False:
                            a[(self.m-1)*(self.order+1)*self.n + k*(self.order+1): (self.m-1)*(self.order+1)*self.n+k*(self.order+1)+self.order + 1] = values
                            A2[k + h*(self.n-1) + (self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n-1) + (self.n-1)*self.k_r] = constraint_data_r[i, h]

                else:
                    # Elsewhere: polynomial of each segment which has time as 0, 1 except initial, final point.
                    end_values = self.poly_coef[h+1, 1]
                    start_values = self.poly_coef[h+1, 0]

                    continuity = False
                    if constraint_data_r[i, h] == 1:
                        # Continuity
                        continuity = True

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is True:
                            a[(i-1)*(self.order+1)*self.n + k * (self.order + 1): (i-1) * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = end_values
                            a[i * (self.order + 1) * self.n + k * (self.order+1): i * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = -start_values
                            A2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r] = 0
                        else:
                            a[(i-1)*(self.order+1)*self.n + k * (self.order + 1): (i-1) * (self.order + 1) * self.n + k * (self.order + 1) + self.order + 1] = end_values
                            A2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n-1) + 2*i*(self.n-1)*self.k_r] = constraint_data_r[i, h]

                    for k in range(0, self.n - 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is False:
                            a[i * (self.order+1) * self.n + k * (self.order+1): i * (self.order+1) * self.n + k * (self.order+1) + self.order + 1] = start_values
                            A2[k + h*(self.n - 1) + 2*i*(self.n -1)*self.k_r + (self.n-1)*self.k_r, :] = a
                            b2[k + h*(self.n - 1) + 2*i*(self.n -1)*self.k_r + (self.n-1)*self.k_r] = constraint_data_r[i, h]

        A2, b2 = self.delete_redundant(A2, b2)
        A2 = matrix(A2)
        b2 = matrix(b2)
        return A2, b2

    def yaw_derivative_constraint(self, current_state):
        """
         derivative constraint of yaw trajectory
        In each segment, start and end of polynomial must satisfy derivative constraint
        This is very important for continuity of trajectory
        """

        constraint_data_psi = np.zeros((self.m, self.k_psi))
        if self.k_psi >= 1:
            constraint_data_psi[0, 0] = 0
            constraint_data_psi[1:self.m, 0] = 1
        if self.k_psi >= 2:
            constraint_data_psi[0, 1] = 0
            constraint_data_psi[1:self.m, 1] = 1

        A3 = np.zeros((2 * self.m * self.k_psi, self.n * (self.order + 1) * self.m))
        b3 = np.ones(2 * self.m * self.k_psi) * 0.001

        for i in range(0, self.m):
            for h in range(0, self.k_psi):
                if i == 0:
                    # Initial yaw_dot, yaw_ddot: First polynomial which has time as 0.
                    values = self.poly_coef[h+1, 0]

                    continuity = False
                    if constraint_data_psi[i, h] == 1:
                        # Continuity
                        continuity = True

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is True:
                            a[i * (self.order + 1) * self.n + (k + 3) * (self.order + 1): i * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            a[(self.m-1) * (self.order + 1) * self.n + (k+3) * (self.order+1): (self.m-1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = -values
                            A3[k + h * 1, :] = a
                            b3[k + h * 1] = 0
                        else:
                            a[i * (self.order + 1) * self.n + (k + 3) * (self.order + 1): i * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            A3[k + h * 1, :] = a
                            b3[k + h * 1] = current_state[h+1][k+3] * np.power(self.time_scaling[0], h + 1)

                    # Final yaw_dot, yaw_ddot: Last polynomial which has time as 1.
                    values = self.poly_coef[h+1, 1]

                    continuity = False
                    if constraint_data_psi[i, h] == 1:
                        # Continuity
                        continuity = True  # True

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is False:
                            a[(self.m - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1):
                              (self.m - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = values
                            A3[k + h * 1 + 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 1 * self.k_psi] = constraint_data_psi[i, h]

                else:
                    # Elsewhere: polynomial of each segment which has time as 0, 1 except initial, final point
                    end_values = self.poly_coef[h+1, 1]
                    start_values = self.poly_coef[h+1, 0]

                    continuity = False
                    if constraint_data_psi[i, h] == 1:
                        # Continuity
                        continuity = True  # True

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is True:
                            a[(i - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1): (i - 1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = end_values
                            a[(i) * (self.order + 1) * self.n + (k + 3) * (self.order + 1): (i) * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = -start_values
                            A3[k + h * 1 + 2 * i * 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 2 * i * 1 * self.k_psi] = 0
                        else:
                            a[(i-1) * (self.order + 1) * self.n + (k + 3) * (self.order + 1): (i-1) * (self.order + 1) * self.n + (k+3) * (self.order + 1) + self.order + 1] = end_values
                            A3[k + h * 1 + 2 * i * 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 2 * i * 1 * self.k_psi] = constraint_data_psi[i, h]

                    for k in range(0, 1):
                        a = np.zeros(self.n * (self.order + 1) * self.m)
                        if continuity is False:
                            a[i * (self.order + 1) * self.n + (k + 3) * (self.order + 1):
                              i * (self.order + 1) * self.n + (k + 3) * (self.order + 1) + self.order + 1] = start_values
                            A3[k + h * 1 + 2 * i * 1 * self.k_psi + 1 * self.k_psi, :] = a
                            b3[k + h * 1 + 2 * i * 1 * self.k_psi + 1 * self.k_psi] = constraint_data_psi[i, h]

        A3, b3 = self.delete_redundant(A3, b3)
        A3 = matrix(A3)
        b3 = matrix(b3)
        return A3, b3

    def maxmin_constraint(self, max_vel, min_vel):
        """
         Maximum and minimum constraint
        It determines maximum and minimum speed of drone when traversing way point.
        """
        # constraint for maximum minimum velocity
        # constraint for start, end point
        G1 = np.zeros((4 * self.m * (self.n-1), self.n * (self.order + 1) * self.m))
        h1 = np.ones((4 * self.m * (self.n-1), 1)) * 0.001

        for i in range(0, self.m):
            if i == 0:
                # At initial and final point, it is already set as equality constraint.
                pass
            else:
                x_sin = np.sign(self.keyframe[0, i] - self.keyframe[0, i-1])
                y_sin = np.sign(self.keyframe[1, i] - self.keyframe[1, i-1])
                z_sin = np.sign(self.keyframe[2, i] - self.keyframe[2, i-1])
                sin_array = np.array([x_sin, y_sin, z_sin])

                x_distance = np.abs(self.keyframe[0, i] - self.keyframe[0, i-1])
                y_distance = np.abs(self.keyframe[1, i] - self.keyframe[1, i-1])
                z_distance = np.abs(self.keyframe[2, i] - self.keyframe[2, i-1])
                distance_array = np.array([x_distance, y_distance, z_distance])

                x_direction = np.cos(self.keyframe[3, i])
                y_direction = np.sin(self.keyframe[3, i])
                direction_array = np.array([x_direction, y_direction, 100])

                end_values = self.poly_coef[1, 1]
                start_values = self.poly_coef[1, 0]
                # Time scaling
                end_values = end_values * np.power(1.0 / self.time_scaling[i - 1], 1)
                start_values = start_values * np.power(1.0 / self.time_scaling[i], 1)

                for k in range(0, self.n-1):
                    g = np.zeros(self.n * (self.order + 1) * self.m)
                    g[(i-1) * (self.order + 1) * self.n + k * (self.order + 1): (i-1) * (self.order + 1) * self.n + k * (
                                self.order + 1) + self.order + 1] = end_values
                    # Maximum constraint
                    G1[(2 * k) + (4 * i * (self.n-1)), :] = g * sin_array[k]
                    #h1[(2 * k) + (4 * i * (self.n-1)), :] = np.max([max_vel * direction_array[k] + 0.1, min_vel * distance_array[k]])
                    h1[(2 * k) + (4 * i * (self.n - 1)), :] = max_vel
                    # Minimum constraint
                    G1[(2 * k + 1) + (4 * i * (self.n-1)), :] = -g * sin_array[k]
                    #h1[(2 * k + 1) + (4 * i * (self.n-1)), :] = -min_vel * distance_array[k]
                    h1[(2 * k + 1) + (4 * i * (self.n - 1)), :] = -min_vel

                    g = np.zeros(self.n * (self.order + 1) * self.m)
                    g[i * (self.order + 1) * self.n + k * (self.order + 1): i * (self.order + 1) * self.n + k * (
                                self.order + 1) + self.order + 1] = start_values
                    # Maximum constraint
                    G1[(2 * k) + (2 * (self.n-1)) + (4 * i * (self.n-1)), :] = g * sin_array[k]
                    #h1[(2 * k) + (2 * (self.n-1)) + (4 * i * (self.n-1)), :] = np.max([max_vel * direction_array[k] + 0.1, min_vel * distance_array[k]])
                    h1[(2 * k) + (2 * (self.n - 1)) + (4 * i * (self.n - 1)), :] = max_vel
                    # Minimum constraint
                    G1[(2 * k + 1) + (2 * (self.n-1)) + (4 * i * (self.n-1)), :] = -g * sin_array[k]
                    #h1[(2 * k + 1) + (2 * (self.n-1)) + (4 * i * (self.n-1)), :] = -min_vel * distance_array[k]
                    h1[(2 * k + 1) + (2 * (self.n - 1)) + (4 * i * (self.n - 1)), :] = -min_vel

        G1, h1 = self.delete_redundant(G1, h1)
        G1 = matrix(G1)
        h1 = matrix(h1)
        return G1, h1

    def corridor_constraint(self, corridor_position, corridor_width, n_intermediate):
        """
         This function is to make corridor between certain segment.
        This can be used for straighting line.
        """
        # x y z corridor constraint. -corridor width < x y z < corridor width
        # size: 3 * 2 * intermediate
        G2 = np.zeros((6 * n_intermediate, self.n * (self.order+1) * self.m))
        h2 = np.ones(6 * n_intermediate) * 0.001

        # unit vector of direction of the corridor
        t_vector = (self.keyframe[0:3, corridor_position[1]] - self.keyframe[0:3, corridor_position[0]])
        t_vector = t_vector / np.linalg.norm(t_vector)

        # intermediate time stamps
        t_intermediate = np.linspace(0, 1, n_intermediate+2)
        t_intermediate = t_intermediate[1: -1]

        rix = self.keyframe[0, corridor_position[0]]
        riy = self.keyframe[1, corridor_position[0]]
        riz = self.keyframe[2, corridor_position[0]]
        ri = np.array([rix, riy, riz])

        for i in range(0, n_intermediate):
            # get polynomial at t intermediate
            values = np.ones(self.order+1)
            for j in range(0, self.order+1):
                values[j] = np.power(t_intermediate[i], self.order-j)

            # it is helper array for computation
            g_helper = np.hstack((t_vector[0] * values, t_vector[1] * values, t_vector[2] * values))
            h_helper = (-rix * t_vector[0])+(-riy * t_vector[1])+(-riz * t_vector[2])
            for k in range(0, self.n-1):
                g = np.zeros(self.n * (self.order + 1) * self.m)
                if k == 0:
                    g_helper2 = np.hstack((values, np.zeros(2 * (self.order + 1))))
                elif k == 1:
                    g_helper2 = np.hstack((np.zeros(self.order + 1), values, np.zeros(self.order + 1)))
                else:
                    g_helper2 = np.hstack((np.zeros(2 * (self.order + 1)), values))
                g[corridor_position[0] * self.n * (self.order + 1): corridor_position[0] * self.n * (
                    self.order + 1) + 3 * (self.order + 1)] = \
                    g_helper2 - t_vector[k] * g_helper
                G2[6 * i + 2 * k, :] = g
                h2[6 * i + 2 * k] = corridor_width + ri[k] + t_vector[k] * h_helper
                G2[6 * i + 2 * k + 1, :] = -g
                h2[6 * i + 2 * k + 1] = corridor_width - ri[k] - t_vector[k] * h_helper

        G2 = matrix(G2)
        h2 = matrix(h2)
        return G2, h2

    def delete_redundant(self, A, B):
        """
         This function is helper function for delete redundant row in matrix.
        When matrix is redundant, in other words when matrix is not full rank,
        it is impossible to use quadratic programming.
        """
        i = 0
        while 1:
            if B[i] == 0.001:
                B = np.delete(B, i, axis=0)
                A = np.delete(A, i, axis=0)
                i = i - 1
            else:
                i = i + 1
            length = len(B)

            if i == length:
                break
        return A, B
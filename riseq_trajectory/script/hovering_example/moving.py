import numpy as np


# This function is to make polynomial which just go upward
# In equation, [x, y, z, psi] = [0, 0, f(t), 0]
# Because drone moves along z-axis, other variable must be zero.
# Return coefficient of [ x(t), y(t), z(t), psi(t) ]
def go_upward(time, distance):
    # Set polynomial order
    # In this case, 6 constraints exist
    # Initial and final position, velocity, acceleration
    # Except final position, all state is zero
    # Able to determine 6 coefficient... z(t) = at^5 + bt^4 + ct^3 + dt^2 + et + f
    order = 5

    # Time to take to move.
    t = time
    # Distance to move
    d = distance

    # Construct A Matrix and b Vector with 6 Constraint
    A = np.zeros((order+1, order+1))
    for i in range(0, 3):
        for j in range(0, 2):
            A[2*i+j] = poly_cc(order, i, j*t)
    b = np.zeros(order+1)
    b[1] = d

    # Solution: z(t) = InvA * b
    sol_z = np.linalg.solve(A, b)

    # Other Solution: x(t), y(t), psi(t) = 0
    # Construct Solution form for differential flatness
    # Solution form : [ x(t), y(t), z(t), psi(t) ]
    other_sol = np.zeros(order+1)
    solution = np.hstack((other_sol, other_sol, sol_z, other_sol))

# This function is to make polynomial which connect initial point with desired coordinate
# TODO
def go_along(time, coordinate):
    pass

# Helper function to get polynomial coefficient
def poly_cc(order, k, t):
    value = np.zeros(order+1)
    # Matrix for computation
    compute_mat = np.eye(order+1)
    for i in range(0, order+1):
        value[i] = np.polyval(np.polyder(compute_mat[i], k), t)
    return value

if __name__ == "__main__":
    go_upward()

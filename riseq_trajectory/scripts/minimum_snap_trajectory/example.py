import numpy as np
import compute_unQP
import compute_cost
import compute_gradient


#TODO: Test every cost function
def main():
    order = 9
    k_r = 4
    waypoint = np.array([[0, 0, 0], [10, 10, 10], [20, 20, 20], [30, 30, 30], [40, 40, 40]])
    m = len(waypoint) - 1
    time = np.ones(m) * 10
    time = [9, 9, 9, 9]
    waypoint = np.transpose(waypoint)
    unqp = compute_unQP.UnConstraintQpMatrix(order, k_r)

    fixed_constraints = 6 * m + 4
    free_constraints = 10 * m - fixed_constraints

    P = unqp.compute_P_1axis(m, time)
    P = np.matrix(P)

    A = unqp.compute_A_global(m, time)
    A = np.matrix(A)

    start = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    goal = [[40, 0, 0, 0, 0], [40, 0, 0, 0, 0], [40, 0, 0, 0, 0]]

    b = unqp.end_derivative_global(waypoint, m, start, goal)
    [dF, dP] = unqp.fixed_free_constraints_global(b, m)

    # dF is fixed constraint
    dF = np.transpose(dF)
    dF = np.matrix(dF)

    # dP is free constraint
    dP = np.transpose(dP)
    dP = np.matrix(dP)

    d = np.zeros((10 * m, 3))
    d = np.matrix(d)
    for i in range(3):
        d[:, i] = np.vstack((dF[:, i], dP[:, i]))

    M = unqp.mapping_matrix_global(m)
    M = np.matrix(M)

    # compute every matrix which is used later
    invA = np.linalg.inv(A)
    R = M.T * invA.T * P * invA * M
    RFP = R[0:fixed_constraints, fixed_constraints:]
    RPP = R[fixed_constraints:, fixed_constraints:]
    invRPP = np.linalg.inv(RPP)
    L = invA * M
    Lpp = L[:, fixed_constraints:]

    '''
    np.savetxt("b.csv", b, delimiter=",")
    np.savetxt("dF.csv", dF, delimiter=",")
    np.savetxt("dP.csv", dP, delimiter=",")
    np.savetxt("d.csv", d, delimiter=",")

    c = M*d
    np.savetxt("c.csv", c, delimiter=",")
    '''

    # set initial guess
    d, dP = initial_guess(d, dF, dP, RFP, invRPP)

    #print dP

    kT = 1
    rate = 0.1
    tolerance = 0.01
    max_iteration = 100

    # gradient
    # free constraints x 3 matrix
    dJd_ddp = np.zeros((free_constraints, 3))
    dJd_ddp = np.matrix(dJd_ddp)
    # 1 x m vector
    dJd_dT = np.zeros(m)
    # 1 x m vector
    dJt_dT = np.zeros(m)

    # J = Jd + kT * Jt
    # Jd is [ 1 x 3 vector ]
    Jd = compute_cost.derivative_cost(P, invA, M, d)
    Jt = compute_cost.time_cost(time)
    J = np.sum(Jd) + kT * Jt
    last_J = J

    solution = np.zeros((10 * m, 3))
    solution = np.matrix(solution)

    for i in range(3):
        solution[:, i] = (invA * M * d[:, i])

    print solution
    # print dP
    #print np.sum(solution[2*(order+1):2*(order+1)+order+1, 0])
    #print np.sum(np.polyder(np.array(solution[1*(order+1):1*(order+1)+order+1, 0]).flatten(), 4))

    for i in range(max_iteration):
        dJd_ddp = compute_gradient.derivative_free_gradient(RFP, RPP, dF, dP, dJd_ddp)
        dJd_dT = compute_gradient.derivative_time_gradient(order, k_r, time, m, solution)

        dJt_dT = compute_gradient.time_gradient(m)

        # update next variable
        for i in range(3):
            dP[:, i] = dP[:, i] - rate * dJd_ddp[:, i]
            d[:, i] = np.vstack((dF[:, i], dP[:, i]))

        time = time - rate * (dJd_dT + kT * dJt_dT)
        A = unqp.compute_A_global(m, time)
        A = np.matrix(A)
        invA = np.linalg.inv(A)
        R = M.T * invA.T * P * invA * M
        RFP = R[0:fixed_constraints, fixed_constraints:]
        RPP = R[fixed_constraints:, fixed_constraints:]

        # cost value
        Jd = compute_cost.derivative_cost(P, invA, M, d)
        Jt = compute_cost.time_cost(time)
        J = np.sum(Jd) + kT * Jt

        if last_J - J < tolerance:
            break

        last_J = J
        #print J

    print time
    for i in range(3):
        solution[:, i] = (invA * M * d[:, i])
    #print solution


def initial_guess(d, dF, dP, RFP, invRPP):
    """
     Function to set initial parameter of free constraints.
    Set free constraints using theory of unconstrained QP without collision cost function.
    """

    for i in range(3):
        dP[:, i] = -invRPP * RFP.T * dF[:, i]
        d[:, i] = np.vstack((dF[:, i], dP[:, i]))

    return d, dP


if __name__ == "__main__":
    main()
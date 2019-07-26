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

from cvxopt import matrix, solvers
import numpy as np
import compute_matrix_unQP


def unconstrained_qp(waypoint, time, intermediate=False):
    """
    Function to solve unconstrained quadratic programming.

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
     When the number of segments is m, in one dimension ( [ x y z ] we have 3 dimension without yaw for square matrix)
    fixed constraint : 2 * m + 4 * ( m + 1) = 6 m + 4
    free constraint : 10 * m - ( 6 m + 4 ) = 4 m - 4

     Note that both the cos-Matrix Q and mapping matrix A only depend on the segment time Ts,i and thus are constant
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


    :param order: order of polynomial
    :param waypoint: point that drone need to traverse
    :param state: drone's state like velocity at end of each segment.
    :param time: segment allocated time
    :return: coefficient of polynomial and cost of function which is minimized by quadratic programming
    """
    # change format of way point
    # m : segment number
    # time scaling : time which is taken for each segment
    order = 9
    n = 3
    m = len(waypoint) - 1
    waypoint = np.delete(waypoint, 3, 1)
    waypoint = np.transpose(waypoint)
    time = np.array(time)

    # fourth derivative for position, second derivative for yaw
    # These are relative to input T,M. Minimum snap equals minimum effort.
    # guarantee continuity until jerk and snap.
    k_r = 4
    unqp = compute_matrix_unQP.UnConstraintQpMatrix(m, waypoint, k_r, time)

    P = unqp.compute_p_1axis()
    A = unqp.compute_A_1axis()
    b = unqp.end_derivative()
    [bF, bP] = unqp.fixed_free_constraints(b)
    M = unqp.mapping_matrix()

    A = matrix(A)
    M = matrix(M)
    invA = matrix(np.linalg.inv(A))
    R = M.T * invA.T * P * invA * M

    RFP = R[0:(6 * m + 4), (6 * m + 4):]
    RPP = R[(6 * m + 4):, (6 * m + 4):]

    invRPP = matrix(np.linalg.inv(RPP))
    X = -invRPP * RFP.T

    cost_sum = 0
    solution = np.zeros((3, 10 * m))
    for i in range(3):
        dF = np.array(bF[i])
        dF = matrix(dF)
        dP = X * matrix(dF)

        d = matrix([dF, dP])
        p = invA * M * d

        solution[i][:] = np.array(p.T)
        # value of cost function which is minimized by optimization
        cost = p.T * P * p
        cost_sum = cost_sum + cost

    return solution, cost_sum

    # np.savetxt("A1.csv", A1, delimiter=",")
    # np.savetxt("A2.csv", A2, delimiter=",")
    # np.savetxt("invA.csv", invA, delimiter=",")
    # np.savetxt("P.csv", P, delimiter=",")
    # np.savetxt("RPP.csv", RPP, delimiter=",")
    # np.savetxt("invRPP.csv", invRPP, delimiter=",")
    # np.savetxt("RFP.csv", RFP, delimiter=",")
    # np.savetxt("R.csv", R, delimiter=",")


if __name__ == "__main__":
    np.array([[0, 0, 0], [1, 5, 5], [20, 20, 20], [30, 30, 30]])
    waypoint = np.array([[0, 0, 0, 0], [1, 5, 5, 10], [20, 20, 20, 20], [30, 30, 30, 30]])
    unconstrained_qp(waypoint, 0)
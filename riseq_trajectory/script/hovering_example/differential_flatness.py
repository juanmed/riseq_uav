#!/usr/bin/env python
import rospy
import numpy as np


'''
Caution!!
Matrix Multiplication(*) must be changed to this form; np.matmul(A, B)
In python2, np.array does not recognize (*) as matrix multiplication, does as element multiplication
'''

# define constants
mass = rospy.get_param("riseq/mass", 1.0)
Ixx = rospy.get_param("riseq/Ixx", 0.0049)
Iyy = rospy.get_param("riseq/Iyy", 0.0049)
Izz = rospy.get_param("riseq/Izz", 0.0049)
I = np.array([[Ixx, 0, 0],
               [0, Iyy, 0],
               [0, 0, Izz]])
invI = np.linalg.inv(I)
gravity = rospy.get_param("riseq/gravity", 9.81) # m/s2
b = 0.01  # air drag/friction force
# c = 0.2 #air friction constant


def get_trajectory(solution, order, time_scaling, ref_time):
    flat_output = compute_output(solution, order, time_scaling, ref_time)
    ref_trajectory = compute_ref(flat_output)
    return ref_trajectory


def compute_output(solution, order, time_scaling, ref_time):
    """
    get coefficient of each trajectory [x y z psi]
    position --> until 4th derivative
    yaw --> until 2nd derivative
    """
    solution = np.hstack(solution)

    # Getting coefficient of position, velocity, acceleration, jerk, snap of x y z
    # In psi case, only position velocity acceleration
    x_coeff = np.array(solution[0 * (order + 1): 1 * (order + 1)])
    y_coeff = np.array(solution[1 * (order + 1): 2 * (order + 1)])
    z_coeff = np.array(solution[2 * (order + 1): 3 * (order + 1)])
    psi_coeff = np.array(solution[3 * (order + 1): 4 * (order + 1)])

    x_dot_coeff = np.polyder(x_coeff)
    y_dot_coeff = np.polyder(y_coeff)
    z_dot_coeff = np.polyder(z_coeff)
    psi_dot_coeff = np.polyder(psi_coeff)

    x_ddot_coeff = np.polyder(x_dot_coeff)
    y_ddot_coeff = np.polyder(y_dot_coeff)
    z_ddot_coeff = np.polyder(z_dot_coeff)
    psi_ddot_coeff = np.polyder(psi_dot_coeff)

    x_dddot_coeff = np.polyder(x_ddot_coeff)
    y_dddot_coeff = np.polyder(y_ddot_coeff)
    z_dddot_coeff = np.polyder(z_ddot_coeff)

    x_ddddot_coeff = np.polyder(x_dddot_coeff)
    y_ddddot_coeff = np.polyder(y_dddot_coeff)
    z_ddddot_coeff = np.polyder(z_dddot_coeff)

    # Considering time scaling
    # Our polynomial has variable t, range [0, 1]
    # But in real, we need polynomial which has variable T, range [0, a]
    # t -> T, at time T, Position is X(T) = x(T/a)
    pos_x = np.polyval(x_coeff, ref_time / time_scaling)
    pos_y = np.polyval(y_coeff, ref_time / time_scaling)
    pos_z = np.polyval(z_coeff, ref_time / time_scaling)
    pos_psi = np.polyval(psi_coeff, ref_time / time_scaling)

    # Apply chain rule
    # Velocity is V(T) = v(T/a) * (1/a)
    vel_x = np.polyval(x_dot_coeff, ref_time / time_scaling) * (1.0 / time_scaling)
    vel_y = np.polyval(y_dot_coeff, ref_time / time_scaling) * (1.0 / time_scaling)
    vel_z = np.polyval(z_dot_coeff, ref_time / time_scaling) * (1.0 / time_scaling)
    vel_psi = np.polyval(psi_dot_coeff, ref_time / time_scaling) * (1.0 / time_scaling)

    # Apply chain rule
    # Acceleration is A(T) = a(T/a) * (1/ag^2)
    acc_x = np.polyval(x_ddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**2)
    acc_y = np.polyval(y_ddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**2)
    acc_z = np.polyval(z_ddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**2)
    acc_psi = np.polyval(psi_ddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**2)

    # Apply chain rule
    # Jerk is J(T) = j(T/a) * (1/a^3)
    jerk_x = np.polyval(x_dddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**3)
    jerk_y = np.polyval(y_dddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**3)
    jerk_z = np.polyval(z_dddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**3)

    # Apply chain rule
    # Snap is S(T) = s(T/a) * (1/a^4)
    snap_x = np.polyval(x_ddddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**4)
    snap_y = np.polyval(y_ddddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**4)
    snap_z = np.polyval(z_ddddot_coeff, ref_time / time_scaling) * (1.0 / time_scaling**4)

    pos = np.array([pos_x, pos_y, pos_z])
    vel = np.array([vel_x, vel_y, vel_z])
    acc = np.array([acc_x, acc_y, acc_z])
    jerk = np.array([jerk_x, jerk_y, jerk_z])
    snap = np.array([snap_x, snap_y, snap_z])
    yaw = pos_psi
    yaw_vel = vel_psi
    yaw_acc = acc_psi

    # This is our flat output [ x y z psi ]
    # plus... derivative of output
    # remember that all this form is np.array
    flat_output = [pos, vel, acc, jerk, snap, yaw, yaw_vel, yaw_acc]
    return flat_output

def RotToRPY_ZYX(R):
    """
        Euler angle convention is ZYX, which means first apply
        rotaion of psi-degrees around Z axis, then rotation of
        theta-degrees around new Y axis, and then rotation of
        phi-degrees around new X axis.
        ** The rotation R received should be from body to world frame. **
    """
    theta = np.arcsin(-1.0 * R[2, 0])
    phi = np.arctan2(R[2, 1] / np.cos(theta), R[2, 2] / np.cos(theta))
    psi = np.arctan2(R[1, 0] / np.cos(theta), R[0, 0] / np.cos(theta))

    return np.array([phi, theta, psi])


def RotToRPY_ZXY(R):
    """
    phi, theta, psi = roll, pitch , yaw
    The euler angle convention used is ZXY. This means: first a rotation of psi-degrees
    around Z axis, then rotation of phi-degress around X axis, and finally rotation of
    theta-degrees around Y axis
        ** The rotation R received should be from world to body frame **
    """
    phi = np.arcsin(R[1, 2])
    theta = np.arctan2(-R[0, 2] / np.cos(phi), R[2, 2] / np.cos(phi))
    psi = np.arctan2(-R[1, 0] / np.cos(phi), R[1, 1] / np.cos(phi))
    return np.array([phi, theta, psi])


def get_x(sigma1):
    return sigma1


def get_y(sigma2):
    return sigma2


def get_z(sigma3):
    return sigma3


def get_psi(sigma4):
    return sigma4


def get_u1(t):
    u1 = mass * np.linalg.norm(t)
    return u1


def get_zb(t):
    zb = t / (np.linalg.norm(t))
    return zb


def get_u1_dot(z_b, j):
    u1_dot = mass * np.dot(z_b, j)
    return u1_dot


def get_t_vector(sigma1, sigma2, sigma3):
    # construct the t vector at each point in time
    t_vec = np.array([sigma1, sigma2, sigma3 + gravity])
    return t_vec


def get_xc(sigma4):
    x_c = np.array([np.cos(sigma4), np.sin(sigma4), 0.0])
    return x_c


def get_yc(sigma4):
    y_c = np.array([-1.0 * np.sin(sigma4), np.cos(sigma4), 0.0])
    return y_c


def get_xb(y_c, z_b):
    a = np.cross(y_c, z_b)
    return a / np.linalg.norm(a)


def get_yb(z_b, x_b):
    a = np.cross(z_b, x_b)
    return a / np.linalg.norm(a)


def get_wx(y_b, j, u_1):
    w_x = -1.0 * mass * np.dot(y_b, j) / u_1
    return w_x


def get_wy(x_b, j, u_1):
    w_y = mass * np.dot(x_b, j) / u_1
    return w_y


def get_wz(psi_rate, x_c, x_b, w_y, y_c, z_b):
    """
        Will compute as wz = (a + b)/c
    """
    a = psi_rate * np.dot(x_c, x_b)
    b = w_y * np.dot(y_c, z_b)
    c = np.linalg.norm(np.cross(y_c, z_b, axis=0))
    w_z = (a + b) / c
    return w_z


def get_wy_dot(x_b, s, u_1_dot, w_y, u_1, w_x, w_z):
    """
        Will use wy_dot = (a + b + c)/d
    """
    a = np.dot(x_b, s)
    b = -2.0 * u_1_dot * w_y / mass
    c = -1.0 * u_1 * w_x * w_z / mass
    d = u_1 / mass
    w_y_dot = (a + b + c) / d
    return w_y_dot


def get_wx_dot(y_b, s, u_1_dot, w_x, u_1, w_y, w_z):
    """
        Will use wx_dot = (a + b + c)/d
    """
    a = -1.0 * np.dot(y_b, s)
    b = -2.0 * u_1_dot * w_x / mass
    c = u_1 * w_y * w_z / mass
    d = u_1 / mass
    w_x_dot = (a + b + c) / d
    return w_x_dot


def get_wz_dot(psi_acc, x_c, x_b, psi_rate, w_z, y_b, w_y, z_b, w_x, y_c, w_y_dot):
    """
        Will compute as w_z_dot = (a+b+c+d+e+f)/g
    """
    a = psi_acc * np.dot(x_c, x_b)
    b = 2.0 * psi_rate * w_z * np.dot(x_c, y_b)
    c = -2.0 * psi_rate * w_y * np.dot(x_c, z_b)
    d = -1.0 * w_x * w_y * np.dot(y_c, y_b)
    e = -1.0 * w_x * w_z * np.dot(y_c, z_b)
    f = w_y_dot * np.dot(y_c, z_b)
    g = np.linalg.norm(np.cross(y_c, z_b, axis=0))
    w_z_dot = (a + b + c + d + e + f) / g
    # print("w_z_dot type is: {}".format(type(w_z_dot)))
    return w_z_dot


# This correspond to [u1, u2, u3]
def get_ux(w_dot_, w_):
    # u_x = I * w_dot_ + np.cross(w_, I * w_, axis=0)
    u_x = np.matmul(I, w_dot_) + np.cross(w_, np.matmul(I, w_), axis=0)
    return u_x


def get_ua(u_1, z_b):
    """
        ua = -g*z_w +u1*z_b/m
    """
    u_a = -gravity * np.array([0.0, 0.0, 1.0]) + u_1 * z_b / mass
    return u_a


def get_ub(w_, M):
    # u_b = invI * (-1.0 * np.cross(w_, I * w_, axis=0) + M)
    u_b = np.matmul(invI, (-1.0 * np.cross(w_, np.matmul(I, w_), axis=0) + M))
    return u_b


def get_uc(w_, ori):
    """
    """
    phi_ = ori[0]
    theta_ = ori[1]
    psi_ = ori[2]

    # orientation rate
    peta = np.array([
        [1.0, np.sin(phi_) * np.tan(theta_), np.cos(phi_) * np.tan(theta_)],
        [0.0, np.cos(phi_), -1.0 * np.sin(phi_)],
        [0.0, np.sin(phi_) / np.cos(theta_), np.cos(phi_) / np.cos(theta_)]])
    u_c = peta * w_

    return u_c


def compute_ref(flat_output):
    """
        Compute all reference states and inputs from the given desired trajectory point using
        differential flatness property.
    """
    # extract all quantities from given trajectory
    pos_traj = flat_output[0]  # 3-vector
    vel_traj = flat_output[1]  # 3-vector
    acc_traj = flat_output[2]  # 3-vector
    jerk_traj = flat_output[3]  # 3-vector
    snap_traj = flat_output[4]  # 3-vector
    yaw_traj = flat_output[5]  # scalar
    yaw_dot_traj = flat_output[6]  # scalar
    yaw_ddot_traj = flat_output[7]  # scalar

    # Now, we must express input and state as flat output
    # get t_vector = [acc_x, acc_y, acc_z + g]
    # get u1 as thrust
    t_vec = get_t_vector(acc_traj[0], acc_traj[1], acc_traj[2])
    u_1 = get_u1(t_vec)

    # Rotation matrix as flat output
    z_b = get_zb(t_vec)
    y_c = get_yc(yaw_traj)
    x_b = get_xb(y_c, z_b)  # get_xb(y_c,z_b)
    y_b = get_yb(z_b, x_b)  # get_yb(z_b,x_b)

    j_ = np.array([jerk_traj[0], jerk_traj[1], jerk_traj[2]])
    w_x = get_wx(y_b, j_, u_1)  # get_wx(y_b,j,u_1)
    w_y = get_wy(x_b, j_, u_1)  # get_wy(x_b,j,u_1)
    x_c = get_xc(yaw_traj)
    w_z = get_wz(yaw_dot_traj, x_c, x_b, w_y, y_c, z_b)  # get_wz(psi_rate,x_c,x_b,w_y,y_c,z_b)
    u_1_dot = get_u1_dot(z_b, j_)  # get_u1_dot(z_b,j)
    s_ = np.array([snap_traj[0], snap_traj[1], snap_traj[2]])
    w_y_dot = get_wy_dot(x_b, s_, u_1_dot, w_y, u_1, w_x, w_z)  # get_wy_dot(x_b,s,u_1_dot,w_y,u_1,w_x,w_z)
    w_x_dot = get_wx_dot(y_b, s_, u_1_dot, w_x, u_1, w_y, w_z)  # get_wx_dot(y_b,s,u_1_dot,w_x,u_1,w_y,w_z)
    w_z_dot = get_wz_dot(yaw_ddot_traj, x_c, x_b, yaw_dot_traj, w_z, y_b, w_y, z_b, w_x, y_c, w_y_dot)
    # get_wz_dot(psi_acc,x_c,x_b,psi_rate,w_z,y_b,w_y,z_b,w_x,y_c,w_y_dot)

    # make angular acceleration vector w_dot
    w_dot_ = np.array([w_x_dot, w_y_dot, w_z_dot])

    # make angular velocity vector w
    w_ = np.array([w_x, w_y, w_z])

    # get vector of torque inputs u2, u3, u4
    u_x = get_ux(w_dot_, w_)  # get_ux(w_dot_,w_)

    # get rotation matrix from base frame to world frame
    # for current desired trajectory point.
    # This matrix represents the orientation of the quadrotor
    R_ = np.vstack((x_b, y_b, z_b)).T

    # Get roll pitch yaw angles assuming ZYX Euler angle convention
    # This means: first rotate psi degrees around Z axis,
    # then theta degrees around Y axis, and lastly phi degrees around X axis

    or_ = RotToRPY_ZYX(R_)  # assuming ZYX Eugler angle convention, so sent matrix should be
    # body - world frame

    #or_ = RotToRPY_ZXY(R_.T)  # assuming ZXY Eugler angle convention, so sent matrix should be
    # world - body frame

    # compute u_a input for system reference
    # can be computed as follows or simply the received acc_traj
    u_a = get_ua(u_1, z_b)  # get_ua(u_1,z_b)

    # compute u_b input for system reference
    # equal angular acceleration, so can be received  w_dot_
    u_b = get_ub(w_, u_x)  # get_ub(w_,M)

    # compute u_c input for system reference
    # get orientation rate
    u_c = get_uc(w_, or_)  # get_uc(w_,ori)

    # we need to return back the 1) reference state vector and 2) reference inputs
    # The reference state vector is : (x,y,z, v_x, v_y, v_z, phi, theta, psi, p, q, r)
    # where x,y,z: are position coordinates
    # v_x, v_y, v_z: are velocities
    # phi, theta, psi: are orientation euler angles as described above
    # p, q, r: are angular velocities in body frame X,Y,Z axis respectively

    # we send the received pos_traj, and vel_traj vectors as the reference pos and vel vectors
    # because that is the result from the differential flatness output selection

    # remember that all this form is np.array
    return [pos_traj, vel_traj, or_, w_, u_a, u_b, u_c, u_1, u_x, R_, acc_traj, jerk_traj, snap_traj, yaw_traj, yaw_dot_traj, yaw_ddot_traj]


import tf
import numpy as np
from pyquaternion import Quaternion
import riseq_common.dyn_utils as dyn_utils


# Orientation control feedback gains
Katt = np.zeros((3,4))
Katt[0][1] = 8.0
Katt[1][2] = 8.0
Katt[2][3] = 8.0

def attitude_controller(Rbw, Rbw_des, Kp = Katt):
    # first transform to world->body frame rotations
    Rwb = Rbw.T
    Rwb_des = Rbw_des.T
    #convert into homogenous transformation
    dummy1 = np.zeros((3,1))
    dummy2 = np.zeros((1,4))
    dummy2[0][3] = 1.0
    Rwb = np.concatenate((Rwb,dummy1), axis = 1)
    Rwb = np.concatenate((Rwb, dummy2), axis = 0)
    Rwb_des = np.concatenate((Rwb_des,dummy1), axis = 1)
    Rwb_des = np.concatenate((Rwb_des, dummy2), axis = 0)
    # transform current and desired orientation to quaternions
    q = tf.transformations.quaternion_from_matrix(Rwb)
    q_des = tf.transformations.quaternion_from_matrix(Rwb_des)
    # CAREFUL! tf uses q = [x y z w], pyquaternion uses q = [w x y z]
    q = [q[3], q[0], q[1], q[2]]  
    q_des = [q_des[3], q_des[0], q_des[1], q_des[2] ]
    # Use an object that lets us apply quaternion operations easily
    q = Quaternion(q)
    q_des = Quaternion(q_des)
    # calculate orientation error
    q_e = 1.0*q.inverse * q_des
    q_e = np.array( [ [q_e[0]], [q_e[1]], [q_e[2]], [q_e[3]]] )#/self.pos_loop_freq # back to np.array ....
    #w_fb = 2.0*np.dot(Kp,q_e)
    
    # calculate angular velocity depending on value of quaternion's real part
    if(q_e[0] >= 0.0):
        w_fb = 2.0*np.dot(Kp,q_e)
    else:
        w_fb = -2.0*np.dot(Kp,q_e)
    return w_fb

def reinit_attitude_controller(Rbw, Rbw_des, Katt = Katt):
    """
    Following Faessler, M., Fontana, F., Forster, C., & Scaramuzza, D. (2015). 
    Automatic Re-Initialization and Failure Recovery for Aggressive Flight with a 
    Monocular Vision-Based Quadrotor. 2015 IEEE International Conference on 
    Robotics and Automation (ICRA), 1722-1729. https://doi.org/10.1109/ICRA.2015.7139420
    """

    e3 = np.array([[0.0],[0.0],[1.0]])  #  z axis of body expressed in body frame
    z = np.dot(Rbw,e3)
    z_des = np.dot(Rbw_des,e3)

    # get angle between that takes from desired z-axis to true body z-axis
    alpha = np.arccos(np.dot(z.T,z_des)[0][0])

    # get normal vector between real,desired z-axis and normalize
    n = np.cross(z,z_des, axis = 0)
    n = n/np.linalg.norm(n)

    # transform axis into real body frame ****
    b_n = np.dot(Rbw.T, n)   # remember "n" is expressed in world frame because z, z_des are too

    # if completely aligned, there should be no rotation
    if( alpha == 0.0):
        qe_rp = Quaternion(1.0,0.0,0.0,0.0)
    else:   
        b_n = np.sin(alpha/2.0)*b_n
        qe_rp = Quaternion( np.cos(alpha/2.0), b_n[0][0], b_n[1][0], b_n[2][0])

    #print("qe_rp: {}".format(qe_rp))

    # recover desired p,q components of angular velocity
    if(qe_rp[0] >= 0.0):
        p_des = 2*Katt[0][1]*qe_rp[1]
        q_des = 2*Katt[1][2]*qe_rp[2]
    else:
        p_des = -2*Katt[0][1]*qe_rp[1]
        q_des = -2*Katt[1][2]*qe_rp[2] 

    # Compute rotation matrix for qe_rp 
    Re_rp = tf.transformations.quaternion_matrix([qe_rp[1],qe_rp[2],qe_rp[3],qe_rp[0]])
    Re_rp = Re_rp[0:3,0:3]

    Re = np.dot(Rbw, Re_rp) #***
    Re_y = np.dot(Re.T, Rbw_des)
    #convert into homogenous transformation
    dummy1 = np.zeros((3,1))
    dummy2 = np.zeros((1,4))
    dummy2[0][3] = 1.0
    Re_y = np.concatenate((Re_y,dummy1), axis = 1)
    Re_y = np.concatenate((Re_y, dummy2), axis = 0)
    
    qe_y = tf.transformations.quaternion_from_matrix(Re_y)
    #print("qe_y: {}".format(qe_y))

    # get desired 'r', (yaw) angular velocity
    we_y = dyn_utils.vex(Re_y)
    r_des = Katt[2][3]*we_y[2][0]
    #print("r_des1: {}".format(r_des))
    r_des2 = Katt[2][3]*qe_y[2]
    #print("r_des2: {}".format(r_des2))

    return np.array([[p_des],[q_des],[r_des2]])
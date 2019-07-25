import numpy as np



def saturate_scalar_minmax(value, max_value, min_value):
    """
    @ description saturation function for a scalar with definded maximum and minimum value
    See Q. Quan. Introduction to Multicopter Design (2017), Ch11.3, page 265 for reference
    """
    mean = (max_value + min_value)/2.0
    half_range = (max_value - min_value)/2.0
    return saturate_vector_dg(value-mean, half_range) + mean


# saturation function for vectors
def saturate_vector_dg(v, max_value):
    """
    @description saturation function for the magnitude of a vector with maximum magnitude 
    and guaranteed direction.
    See Q. Quan. Introduction to Multicopter Design (2017), Ch. 10.2 for reference
    """
    mag = np.linalg.norm(v)
    if( mag < max_value):
        return v
    else:
        return np.dot(v/mag,max_value)  # return vector in same direction but maximum possible magnitude

def vex(mx):
    """
    Take skew-symmetric matrix and turn it into vector
    """
    v = np.array([[0.0],[0.0],[0.0]])
    v[2][0] = 0.5*(mx[1][0] - mx[0][1])
    v[1][0] = 0.5*(mx[0][2] - mx[2][0])
    v[0][0] = 0.5*(mx[2][1] - mx[1][2])

    return v

def vex2(mx):
    v = np.array([[0.0],[0.0],[0.0]])
    v[2][0] = mx[1][0]
    v[1][0] = mx[0][2]
    v[0][0] = mx[2][1]

    return v  

def to_homogeneous_transform(R):
    """
    """
        #convert into homogenous transformation
    dummy1 = np.zeros((3,1))
    dummy2 = np.zeros((1,4))
    dummy2[0][3] = 1.0
    R = np.concatenate((R,dummy1), axis = 1)
    R = np.concatenate((R, dummy2), axis = 0)
    return R

def rotation_distance( R1, R2):
    """
    @description Measure the angle to rotate from R1 to R2
    at http://www.boris-belousov.net/2016/12/01/quat-dist/
    @R1 3x3 np.array, must be a member of the Special Orthogonal group SO(3)
    @R2 3x3 np.array, must be a member of the Special Orthogonal group SO(3)
    @return absolute value of the angle between R1 and R2
    """
    Rd = np.dot(R1.T, R2)       # get difference rotation matrix
    angle = np.arccos((np.trace(Rd) -1.0 )/2.0)
    return np.abs(angle)
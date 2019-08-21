import numpy as np



def saturate_scalar_minmax(value, max_value, min_value):
    """
    Saturation function for a scalar with defineded maximum and minimum value
    See Q. Quan. Introduction to Multicopter Design (2017), Ch11.3, page 265 for reference

    Args:
        value (float): actual value of variable to saturate
        max_value (float): maximum allowable value of variable
        min_value (float): min_valule allowable value of variable

    Returns:
        Saturated value of variable
    """
    mean = (max_value + min_value)/2.0
    half_range = (max_value - min_value)/2.0
    return saturate_vector_dg(value-mean, half_range) + mean


# saturation function for vectors
def saturate_vector_dg(v, max_value):
    """
    Saturation function for the magnitude of a vector with maximum magnitude 
    and guaranteed direction.
    See Q. Quan. Introduction to Multicopter Design (2017), Ch. 10.2 for reference

    Args:
        v (np.array like): vector to saturate
        max_value (float): maximum value for the magnitude of vector v. Must be positive.
    Returns:
        Saturated vector v if its magnitude is greater than max_value. The same
        vector otherwise.
    """
    mag = np.linalg.norm(v)
    if( mag < abs(max_value)):
        return v
    else:
        return np.dot(v/mag,max_value)  # return vector in same direction but maximum possible magnitude

def vex(mx):
    """
    Take skew-symmetric matrix and turn it into vector. In other words, map R3x3 to R3.
    Defined by http://www.petercorke.com/RTB/r9/html/vex.html
    Args:
        mx (np.array): 3x3 skew-symmetric matrix

    Returns:
        3x1 vector 
    """
    v = np.array([[0.0],[0.0],[0.0]])
    v[2][0] = 0.5*(mx[1][0] - mx[0][1])
    v[1][0] = 0.5*(mx[0][2] - mx[2][0])
    v[0][0] = 0.5*(mx[2][1] - mx[1][2])

    return v

def vex2(mx):
    """
    Take skew-symmetric matrix and turn it into vector. In other words, map R3x3 to R3.
    Defined by http://www.petercorke.com/RTB/r9/html/vex.html but instead of taking
    the mean of identical elements, it simply takes on of them and returns.
    Args:
        mx (np.array): 3x3 skew-symmetric matrix

    Returns:
        3x1 vector
    """

    v = np.array([[0.0],[0.0],[0.0]])
    v[2][0] = mx[1][0]
    v[1][0] = mx[0][2]
    v[0][0] = mx[2][1]

    return v  

def to_homogeneous_transform(R):
    """
    Convert a 3x3 matrix into a 4x4 matrix by adding 0-element row and column,
    and making the 4,4 element equal to 1.
    Args:
        R (3x3 np.array): matrix to transform. Usually a rotation matrix.
    Returns:
        4x4 np.array 
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
    Measure the angle to rotate from R1 to R2 obtained from
    at http://www.boris-belousov.net/2016/12/01/quat-dist/
    
    Args:
        R1 (3x3 np.array): must be a member of the Special Orthogonal group SO(3)
        R2 (3x3 np.array): must be a member of the Special Orthogonal group SO(3)
    
    Returns: 
        absolute value of the angle between R1 and R2
    """
    Rd = np.dot(R1.T, R2)       # get difference rotation matrix
    angle = np.arccos((np.trace(Rd) -1.0 )/2.0)
    return np.abs(angle)
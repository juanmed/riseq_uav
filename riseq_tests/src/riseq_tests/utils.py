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

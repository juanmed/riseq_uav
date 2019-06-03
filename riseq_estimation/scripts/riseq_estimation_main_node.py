#!/usr/bin/env python

import rospy

from riseq_estimation_DCM import DirectionCosineMatrix
from riseq_estimation_LGEKF import LieGroupExtendedKalmanFilter

if __name__ == "__main__":
    method = 'LG-EKG'

    if method == 'DCM':
        estimator = DirectionCosineMatrix()
    elif method == 'LG-EKF':
        estimator = LieGroupExtendedKalmanFilter()

    # Run the estimator node
    try:
        while not rospy.is_shutdown():
            estimator.loop()
    except rospy.ROSInterruptException:
        pass

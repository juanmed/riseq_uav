#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry


class Estimator:
    def __init__(self):
        """
        This class is rapper class for ground truth.
        It changes topic name for riseq system.
        """
        model_name = rospy.get_param("model_name", "firefly")

        # Create subscriber and publisher
        rospy.Subscriber("/" + model_name + "/ground_truth/odometry", Odometry, self.groundtruth_cb)
        self.state_pub = rospy.Publisher("riseq/estimator/uav_estimated_state", Odometry, queue_size=10)

    def groundtruth_cb(self, msg):
        odometry = Odometry()
        odometry = msg
        self.state_pub.publish(odometry)


if __name__ == "__main__":
    # Init Node
    rospy.init_node('riseq_estimator', anonymous=True)

    # Make Class
    estimator = Estimator()
    rospy.spin()
    try:
        rospy.loginfo("UAV estimator created")
    except rospy.ROSInterruptException:
        print("ROS Terminated.")
#!/usr/bin/env python

import rospy
import tf

if __name__ == "__main__":
    try:
        rospy.init_node('riseq_gazebo_tf_publisher')
        br = tf.TransformBroadcaster()
        r = rospy.Rate(1000)

        q = tf.transformations.quaternion_from_euler(0, 1.57, -1.57, axes="szyx")
        while not rospy.is_shutdown():
            br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "local_origin", "map")
            br.sendTransform((0, 0, 0.1), (q[0], q[1], q[2], q[3]), rospy.Time.now(), "r200", "fcu")
            r.sleep()

    except rospy.ROSInterruptException:
        pass

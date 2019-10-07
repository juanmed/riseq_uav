#!/usr/bin/env python

import rospy
import tf

if __name__ == "__main__":
    try:
        rospy.init_node('riseq_gazebo_tf_publisher')
        br = tf.TransformBroadcaster()
        r = rospy.Rate(1000)

        r200 = tf.transformations.quaternion_from_euler(0, 1.57, -1.57, axes="szyx")
        fcu = tf.transformations.quaternion_from_euler(3.14, 0, 0)
        while not rospy.is_shutdown():
            br.sendTransform((0, 0, 0), (fcu[0], fcu[1], fcu[2], fcu[3]), rospy.Time.now(), "fcu", "fcu_frd")
            br.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "local_origin", "map")
            br.sendTransform((0, 0, 0.1), (r200[0], r200[1], r200[2], r200[3]), rospy.Time.now(), "r200", "fcu")
            r.sleep()

    except rospy.ROSInterruptException:
        pass

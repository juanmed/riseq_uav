#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseStamped

def publish_fg_uav_pose():

    # init node
    rospy.init_node('riseq_fg_true_pose_publisher', anonymous = True)
    rospy.loginfo("Flightgoggles True Pose Publisher Started")

    # create topic and topic publisher
    pose_publisher = rospy.Publisher('riseq/tests/uav_fg_true_pose', PoseStamped, queue_size = 10)

    # start a tf subscriber
    tf_subscriber = tf.TransformListener()

    # wait for transform tree to complete... 
    # the first time might be problematic since transforms might not be already
    # published
    tf_subscriber.waitForTransform("/world", "uav/imu", rospy.Time(), rospy.Duration(4.0))

    # Set true state publish rate
    try:
        publish_rate = int(rospy.get_param("riseq/true_state_publish_rate"))
    except:
        print("/riseq/true_state_publish_rate parameter is unavailable")
        print("Will publish true state at 100hz")
        publish_rate = 100
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():

        try:
            # get time of latest available transform
            latest_time = tf_subscriber.getLatestCommonTime('/world', 'uav/imu')

            # get transform -translation,rotation- at latest time
            (uav_t, uav_r) = tf_subscriber.lookupTransform('/world', 'uav/imu', latest_time)

            # create and populate PoseStamped message
            pose_msg = PoseStamped()

            # Fill Header first
            pose_msg.header.stamp = rospy.Time.now() #latest_time , for sync
            pose_msg.header.frame_id = 'uav/imu'

            # Fill Translation
            pose_msg.pose.position.x = uav_t[0]
            pose_msg.pose.position.y = uav_t[1]
            pose_msg.pose.position.z = uav_t[2]

            # Fill Orientation
            pose_msg.pose.orientation.x = uav_r[0]
            pose_msg.pose.orientation.y = uav_r[1]
            pose_msg.pose.orientation.z = uav_r[2]
            pose_msg.pose.orientation.w = uav_r[3]

            pose_publisher.publish(pose_msg)
            rospy.loginfo(pose_msg)

        except (tf.LookupException, tf.ConnectivityException): #tf.ExtrapolationException
            rospy.loginfo('tf connection error!')
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_fg_uav_pose()
    except rospy.ROSInterruptException:
        print('ROS Terminated')
        pass

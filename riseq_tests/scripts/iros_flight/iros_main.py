#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
import tf
import numpy as np


current_state = State()
current_pose = PoseStamped()

gate_count = 0
wp = [(0, 0, 2), (8.5, 0, 2), (8.5, 4, 2), (0, 4, 2), (-6.5, 4, 2), (-6.5, 0, 2), (0, 0, 2)]
heading = [0, 0, np.pi/2, np.pi, np.pi, np.pi * 3/2, 0]

def state_cb(msg):
    global current_state
    current_state = msg


def pose_cb(msg):
    global current_pose
    current_pose = msg


if __name__ == "__main__":
    global current_state
    rospy.init_node('offb_node', anonymous=True)

    # Create Subscriber
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)

    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    print("Publisher and Subscriber Created")
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)

    print("Clients Created")
    rate = rospy.Rate(20)

    while (not current_state.connected):
        print(current_state.connected)
        rate.sleep()

    print("Creating pose")

    goal_pose = PoseStamped()
    # set position here
    goal_pose.pose.position.x = 0
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = 2

    for i in range(100):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    print("OFFBOARD and ARMING")
    resp1 = set_mode_client(0, offb_set_mode.custom_mode)
    arm_client_1 = arming_client(arm_cmd.value)

    while current_state.mode != "OFFBOARD" or not current_state.armed:
        rospy.sleep(5.0)

    for i in range(100):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    wp_len = len(wp)
    wp_index = 0
    while not rospy.is_shutdown():
        q = tf.transformations.quaternion_from_euler(0, 0, heading[wp_index], axes='sxyz')

        local_pos_pub.publish(goal_pose)
        if np.linalg.norm((current_pose.pose.position.x - goal_pose.pose.position.x, current_pose.pose.position.y - goal_pose.pose.position.y)) < 0.3:
            wp_index = wp_index + 1
            if wp_index == wp_len:
                break
        goal_pose.pose.position.x = wp[wp_index][0]
        goal_pose.pose.position.y = wp[wp_index][1]
        goal_pose.pose.position.z = wp[wp_index][2]
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        rospy.loginfo(goal_pose)
        # print current_state
        rate.sleep()

    print("Return")
    goal_pose.pose.position.x = 0
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = 0.1
    for i in range(100):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    print("landing")
    land_cmd = CommandTOL()
    landing_client(0.0, 0.0, 0.0, 0.0, 0.0)

    rospy.sleep(5)

    print("disarming")
    arm_cmd.value = False
    arm_client_1 = arming_client(arm_cmd.value)
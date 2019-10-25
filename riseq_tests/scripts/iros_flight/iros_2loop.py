#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
import tf
import numpy as np


current_state = State()
current_pose = PoseStamped()
computed_gate_pose = PoseStamped()
solvepnp_gate_pose = PoseStamped()


def state_cb(msg):
    global current_state
    current_state = msg
    print "callback"


def pose_cb(msg):
    """
    callback function to receive current drone's pose.
    """
    global current_pose
    current_pose = msg


def computed_gate_cb(msg):
    """
    callback function to receive gate x y z position
    :param msg: gate position
    """
    global computed_gate_pose
    computed_gate_pose = msg


if __name__ == "__main__":
    global current_state
    global current_pose
    global computed_gate_pose
    rospy.init_node('iros_node', anonymous=True)

    # Create Subscriber
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/riseq/perception/computed_position", PoseStamped, computed_gate_cb)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    print("Publisher and Subscriber Created")

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
    print("Clients Created")
    rate = rospy.Rate(20)

    while not current_state.connected:
        print(current_state.connected)
        rate.sleep()

    print("Creating pose")
    goal_pose = PoseStamped()
    # set position here
    goal_pose.pose.position.x = 0
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = 1.5

    for i in range(50):
        local_pos_pub.publish(goal_pose)
        rate.sleep()
    print("OFFBOARD")
    #for i in range(100):
    #    local_pos_pub.publish(goal_pose)
    #    rate.sleep()

    print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    resp1 = set_mode_client(0, offb_set_mode.custom_mode)
    arm_client_1 = arming_client(arm_cmd.value)

    #while current_state.mode != "OFFBOARD" or not current_state.armed:
    #    print "not arming"
    #    rospy.sleep(1.0)

    for i in range(200):
        local_pos_pub.publish(goal_pose)
        rate.sleep()

    loop = 0
    step = 0    # 0 : go to gate, 1 : go 1m, 2 : back 1m, 3 : go origin
    heading = 0
    goal_update = True
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        if goal_update:
            if step == 0:
                if computed_gate_pose.header.stamp - rospy.Time.now() > rospy.Duration(1.0):
                    print "Cannot find gate"
                    goal_pose.pose.position.x = 0
                    goal_pose.pose.position.y = 0
                    goal_pose.pose.position.z = 1.5
                    break

                if loop == 0:
                    goal_pose.pose.position.x = current_pose.pose.position.x + computed_gate_pose.pose.position.x
                    goal_pose.pose.position.y = current_pose.pose.position.y + computed_gate_pose.pose.position.y
                    goal_pose.pose.position.z = current_pose.pose.position.z + computed_gate_pose.pose.position.z
                else:
                    goal_pose.pose.position.x = current_pose.pose.position.x - computed_gate_pose.pose.position.x
                    goal_pose.pose.position.y = current_pose.pose.position.y - computed_gate_pose.pose.position.y
                    goal_pose.pose.position.z = current_pose.pose.position.z + computed_gate_pose.pose.position.z
                print "go to gate"
            elif step == 1:
                if loop == 0:
                    goal_pose.pose.position.x = current_pose.pose.position.x + 0.5
                else:
                    goal_pose.pose.position.x = current_pose.pose.position.x - 0.5
                print "pass gate"
            elif step ==2:
                if loop == 0:
                    goal_pose.pose.position.x = current_pose.pose.position.x - 0.5
                else:
                    goal_pose.pose.position.x = current_pose.pose.position.x + 0.5
                print "back to gate"
                start_time = rospy.Time.now()
            elif step == 3:
                heading = heading + np.pi
                if heading >= 2 * np.pi:
                    heading = heading - 2 * np.pi

                q = tf.transformations.quaternion_from_euler(0, 0, heading, axes='sxyz')
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]		
                print "heading change"
            elif step == 4:
                goal_pose.pose.position.x = 0
                goal_pose.pose.position.y = 0
                goal_pose.pose.position.z = 1.5
                print "go origin"

            goal_update = False

        if step == 3:
            if (rospy.Time.now() - start_time) >= rospy.Duration(4.0):
                goal_update = True
                step = step + 1
        else:
            if np.linalg.norm((current_pose.pose.position.x - goal_pose.pose.position.x, current_pose.pose.position.y - goal_pose.pose.position.y)) < 0.2:
                goal_update = True
                step = step + 1
                if step == 5:
                    step = 0
                    loop = loop + 1
                    if loop == 2:
                        break

        goal_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(goal_pose)
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

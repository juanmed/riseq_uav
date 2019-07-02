#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL

current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg
    print "callback"


if __name__ == "__main__":
    global current_state
    rospy.init_node('offb_node', anonymous=True)
    rospy.Subscriber("mavros/state", State, state_cb)
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
    pose = PoseStamped()
    # set position here
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1

    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    print("Creating Objects for services")
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    last_request = rospy.Time.now()
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # print(current_state)
        if (current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            resp1 = set_mode_client(0, offb_set_mode.custom_mode)
            if resp1.mode_sent:
                print ("Offboard enabled")
            last_request = rospy.Time.now()
        elif (not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
            arm_client_1 = arming_client(arm_cmd.value)
            if arm_client_1.success:
                print("Vehicle armed")
            last_request = rospy.Time.now()

        local_pos_pub.publish(pose)
        # print current_state
        rate.sleep()

        if rospy.Time.now() - start_time > rospy.Duration(15.0):
            break

    print("Return")
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0.1
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    print("landing")
    land_cmd = CommandTOL()
    landing_client(0.0, 0.0, 0.0, 0.0, 0.0)

    rospy.sleep(5)

    print("disarming")
    arm_cmd.value = False
    arm_client_1 = arming_client(arm_cmd.value)


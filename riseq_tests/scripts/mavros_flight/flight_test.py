#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from nav_msgs.msg import Path
import tf
import numpy as np

from collections import deque


current_state = State()
waypoint = []
receive_path = False
position = np.zeros(3) # [ x y z ]

avg_samples = 10
monocular_waypoints = deque( maxlen = avg_samples )
uav_positions = deque( maxlen = avg_samples )

def state_cb(msg):
    global current_state
    current_state = msg


def path_cb(msg):
    global waypoint
    global receive_path
    receive_path = True
    # m is segment
    m = len(msg.poses) - 1
    waypoint = np.zeros((m + 1, 4))
    for i in range(0, m + 1):
        waypoint[i][0] = msg.poses[i].pose.position.x
        waypoint[i][1] = msg.poses[i].pose.position.y
        waypoint[i][2] = msg.poses[i].pose.position.z
        phi, theta, psi = tf.transformations.euler_from_quaternion(
            [msg.poses[i].pose.orientation.x, msg.poses[i].pose.orientation.y, msg.poses[i].pose.orientation.z,
             msg.poses[i].pose.orientation.w], axes='sxyz')
        waypoint[i][3] = psi

def monocular_waypoint_cb(msg):
    """
    """
    global monocular_waypoints
    wp = np.zeros((3,1))
    if(len(msg.poses)>0):
        wp[0][0] = msg.poses[0].pose.position.x
        wp[1][0] = msg.poses[0].pose.position.y
        wp[2][0] = msg.poses[0].pose.position.z
        monocular_waypoints.append(wp)



def pose_cb(msg):
    global position
    global uav_positions
    position[0] = msg.pose.position.x
    position[1] = msg.pose.position.y
    position[2] = msg.pose.position.z


    pos = np.zeros((3,1))
    pos[0][0] = msg.pose.position.x
    pos[1][0] = msg.pose.position.y
    pos[2][0] = msg.pose.position.z    
    uav_positions.append(pos)


if __name__ == "__main__":
    global current_state
    global waypoint
    global receive_path
    global position


    global monocular_waypoints
    global uav_positions
    global avg_samples
    global send_mono_waypoint
    send_mono_waypoint = True
    param = 0.0
    param_step = 0.1
    global avg_samples
    mode_wait_time = rospy.Duration(5.0)
    step = 1

    rospy.init_node('Flight_node', anonymous=True)

    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("riseq/planning/uav_waypoint", Path, path_cb)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_cb)
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    att_raw_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

    rospy.Subscriber("riseq/perception/uav_mono_waypoint", Path, monocular_waypoint_cb)

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)

    rate = rospy.Rate(20)

    while not current_state.connected:
        print "connected is False"
        rate.sleep()

    pose = PoseStamped()
    # set position here
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    att = AttitudeTarget()

    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True

    # mode : [ Start, Window, Pole, Pipe, Tree, Net, Wind ]
    # pub_mode : [ Position, Attitude ]
    mode = "Start"
    pub_mode = "Position"
    last_request = rospy.Time.now()
    last_position = np.zeros(3)  # [ x, y, z ]
    iteration = 0

    height = 1.5
    width = 1
    print "Start loop"
    while not rospy.is_shutdown():
        # print(current_state)
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            resp1 = set_mode_client(0, offb_set_mode.custom_mode)
            if resp1.mode_sent:
                print "Offboard enabled"
            last_request = rospy.Time.now()
        elif not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            arm_client_1 = arming_client(arm_cmd.value)
            if arm_client_1.success:
                print "Vehicle armed"
            last_request = rospy.Time.now()

        if pub_mode == "Position":
            local_pos_pub.publish(pose)
        elif pub_mode == "Attitude":
            att_raw_pub.publish(att)
        # print current_state
        rate.sleep()

        if current_state.mode == "OFFBOARD" and current_state.armed and ((rospy.Time.now() - last_request) > mode_wait_time):
            if mode == "Start":
                if iteration == 0:
                    print "Start Mode"
                    pub_mode = "Position"
                    iteration = iteration + 1
                    rospy.set_param("riseq/planning/goalpoint", "none")

                if rospy.Time.now() - last_request < rospy.Duration(15.0):
                    if iteration == 1:
                        print "Start Mode: hovering"
                        iteration = iteration + 1
                    pose.pose.position.z = height * (
                                rospy.Time.now() - last_request - rospy.Duration(5.0)) / rospy.Duration(10.0)
                elif rospy.Time.now() - last_request < rospy.Duration(20.0):
                    if iteration == 2:
                        print "Turn Right"
                        iteration = iteration + 1
                    q = tf.transformations.quaternion_from_euler(0, 0, -1.5707 *  (rospy.Time.now() - last_request - rospy.Duration(15.0)) / rospy.Duration(5.0))
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                elif rospy.Time.now() - last_request < rospy.Duration(25.0):
                    if iteration == 3:
                        print "Turn Origin"
                        iteration = iteration + 1
                    q = tf.transformations.quaternion_from_euler(0, 0, -1.5707 + 1.5707 *  (rospy.Time.now() - last_request - rospy.Duration(20.0)) / rospy.Duration(5.0))
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                elif rospy.Time.now() - last_request < rospy.Duration(35.0):
                    if iteration == 4:
                        print "Start Mode: go right"
                        iteration = iteration + 1
                    pose.pose.position.x = 0
                    pose.pose.position.y = 0 - 3 * (rospy.Time.now() - last_request - rospy.Duration(25.0)) / rospy.Duration(10.0)
                    pose.pose.position.z = height
                elif rospy.Time.now() - last_request < rospy.Duration(45.0):
                    if iteration == 5:
                        print "Start Mode: go forward to find window"
                        last_position[0] = position[0]
                        last_position[1] = position[1]
                        last_position[2] = position[2]
                        iteration = iteration + 1
                    pose.pose.position.x = last_position[0] + 3 * (rospy.Time.now() - last_request - rospy.Duration(35.0)) / rospy.Duration(10.0)
                    pose.pose.position.y = last_position[1]
                    pose.pose.position.z = height
                    q = tf.transformations.quaternion_from_euler(0, 0, 1.5707/2 * (
                                rospy.Time.now() - last_request - rospy.Duration(35.0)) / rospy.Duration(10.0))
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                else:
                    print "Start Mode Over"
                    mode = "Window"
                    iteration = 0
                    last_request = rospy.Time.now()

            elif mode == "Window":
                if iteration == 0:
                    print "Window mode"
                    #pub_mode = "Attitude"
                    iteration = iteration + 1
                    calculate_waypoint = True
                    #mode_wait_time = rospy.Duration(1.0)

                '''
                Juan's code for detecting window, generating trajectory, control drone.
                '''

                rospy.set_param('riseq/monocular_cv', 'window')


                if ( send_mono_waypoint ) :
                    if( calculate_waypoint):
                        #print("Calculate waypoint {}".format(step))
                        if (len(monocular_waypoints) == avg_samples):

                            # get avg waypoint
                            avg_mono_waypoint = np.zeros((3,1))
                            for wp in monocular_waypoints:
                                avg_mono_waypoint = avg_mono_waypoint + wp/avg_samples

                            # get avg uav position
                            avg_uav_position = np.zeros((3,1))
                            for pos in uav_positions:
                                avg_uav_position = avg_uav_position + pos/avg_samples

                            avg_waypoint = (avg_mono_waypoint * param) + avg_uav_position
                            calculate_waypoint = False
                            param = param + param_step

                            pose.pose.position.x = avg_waypoint[0][0] + 1.1
                            pose.pose.position.y = avg_waypoint[1][0] + 0.0
                            pose.pose.position.z = avg_waypoint[2][0] + 0.0

                    else:
                        avg_waypoint= (avg_mono_waypoint * param) + avg_uav_position
                        #print(avg_waypoint)
                        param = param + param_step

                        pose.pose.position.x = avg_waypoint[0][0] 
                        pose.pose.position.y = avg_waypoint[1][0]
                        pose.pose.position.z = avg_waypoint[2][0]

                    if( param >= 1.0):
                        send_mono_waypoint = False

                else:
                    # get avg uav position
                    avg_uav_position = np.zeros((3,1))
                    for pos in uav_positions:
                        avg_uav_position = avg_uav_position + pos/avg_samples

                    goal_point = np.array([[pose.pose.position.x], [pose.pose.position.y], [pose.pose.position.y]])
                    error = goal_point - avg_uav_position
                    error = np.linalg.norm(error)

                    #print("Error: {}".format(error))
                    #print("Step: {}".format(step))
                    if error < 0.2:
                        if (step == 0):
                            step = step + 1
                            send_mono_waypoint = True
                            calculate_waypoint = True
                            param = 0.0
                            param_step = 0.5  # distance should be shorter so go fasterS
                        else:

                            print "Window mode Over"
                            mode = "Pole"
                            iteration = 0
                            last_request = rospy.Time.now()
                            step = 0
                    else:
                        quien = "waypoint" if (np.linalg.norm(avg_uav_position) < np.linalg.norm(goal_point)) else "position"
                        #print("Mayor es {}".format(quien))

            elif mode == "Pole":
                if iteration == 0:
                    print "Pole mode"
                    rospy.set_param("riseq/planning/goalpoint", "random")
                    pub_mode = "Position"
                    last_position[0] = position[0]
                    last_position[1] = position[1]
                    last_position[2] = position[2]
                    pose.pose.position.x = last_position[0] + 0.5
                    pose.pose.position.y = last_position[1]
                    pose.pose.position.z = last_position[2]
                    q = tf.transformations.quaternion_from_euler(0, 0, 0)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    iteration = iteration + 1
                    mode_wait_time = rospy.Duration(5.0)

                # To Construct Octomap, it moves left and right for 30 seconds
                if rospy.Time.now() - last_request < rospy.Duration(15.0):
                    if iteration == 1:
                        print "go right for octomap"
                        last_position[0] = position[0]
                        last_position[1] = position[1]
                        last_position[2] = position[2]
                        iteration = iteration + 1
                    pose.pose.position.y = last_position[1] - width * (
                                rospy.Time.now() - last_request - rospy.Duration(5.0)) / rospy.Duration(10.0)
                elif rospy.Time.now() - last_request < rospy.Duration(25.0):
                    if iteration == 2:
                        print "go left for octomap"
                        last_position[0] = position[0]
                        last_position[1] = position[1]
                        last_position[2] = position[2]
                        iteration = iteration + 1
                    pose.pose.position.y = last_position[1] + 2 * width * (
                                rospy.Time.now() - last_request - rospy.Duration(15.0)) / rospy.Duration(10.0)
                elif rospy.Time.now() - last_request < rospy.Duration(35.0):
                    if iteration == 3:
                        print "go origin"
                        last_position[0] = position[0]
                        last_position[1] = position[1]
                        last_position[2] = position[2]
                        iteration = iteration + 1
                    pose.pose.position.y = last_position[1] - 1 * width * (
                                rospy.Time.now() - last_request - rospy.Duration(25.0)) / rospy.Duration(10.0)
                else:

                    print "follow path"
                    if not receive_path:
                        print "no map"
                        break
                    else:
                        if iteration == 4:
                            last_position[0] = position[0]
                            last_position[1] = position[1]
                            last_position[2] = position[2]
                        if np.sqrt((last_position[0] - position[0])**2 + (last_position[1] - position[1])**2) > 18:
                            iteration = 0
                            last_request = rospy.Time.now()
                            mode = "Pipe mode Over"
                            break
                        if len(waypoint) < 3:
                            pose.pose.position.x = waypoint[-1][0]
                            pose.pose.position.y = waypoint[-1][1]
                        else:
                            pose.pose.position.x = waypoint[2][0]
                            pose.pose.position.y = waypoint[2][1]

            '''
            elif mode == "Pipe":

                if iteration == 0:
                    print "Pipe mode"
                    iteration = iteration + 1

            '''

    print "Return"
    pose.pose.position.z = 0.1
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    print "landing"
    land_cmd = CommandTOL()
    landing_client(0.0, 0.0, 0.0, 0.0, 0.0)

    rospy.sleep(5.0)

    print("disarming")
    arm_cmd.value = False
    arm_client_1 = arming_client(arm_cmd.value)

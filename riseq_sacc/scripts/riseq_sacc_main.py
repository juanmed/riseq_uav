#!/usr/bin/env python
"""
author:  Eugene Auh
version: 0.1.0
brief: 

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the ""Software""), to deal in the 
Software without restriction, including without limitation the rights to use, copy, 
modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the 
following conditions:
The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.
THE SOFTWARE IS PROVIDED *AS IS*, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE 
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import rospy
import numpy as np
import geopy.distance
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget, HomePosition
from mavros_msgs.srv import CommandHome, CommandBool, SetMode, CommandTOL

current_state = State()
current_position = NavSatFix()
home = HomePosition()

wp0 = GlobalPositionTarget()
wp1 = GlobalPositionTarget()
wp2 = GlobalPositionTarget()
wp3 = GlobalPositionTarget()
wp4 = GlobalPositionTarget()

wp = GlobalPositionTarget()
wp_target = GlobalPositionTarget()
wp_avoidance = GlobalPositionTarget()
wp_helical = GlobalPositionTarget()

wp0.coordinate_frame = GlobalPositionTarget().FRAME_GLOBAL_INT
wp0.type_mask = GlobalPositionTarget().IGNORE_VX + GlobalPositionTarget().IGNORE_VY + GlobalPositionTarget().IGNORE_VZ + GlobalPositionTarget().IGNORE_AFX + GlobalPositionTarget().IGNORE_AFY + GlobalPositionTarget().IGNORE_AFZ + GlobalPositionTarget().FORCE + GlobalPositionTarget().IGNORE_YAW_RATE
wp0.latitude  = 37.565011
wp0.longitude = 126.628919
wp0.altitude  = 5.0
wp0.yaw       = np.pi/180.0 * 117.0

wp1.coordinate_frame = wp0.coordinate_frame
wp1.type_mask = wp0.type_mask
wp1.latitude  = 37.566025
wp1.longitude = 126.628206
wp1.altitude  = 100.0
wp1.yaw       = np.pi/180.0 * 117.0

wp2.coordinate_frame = wp0.coordinate_frame
wp2.type_mask = wp0.type_mask
wp2.latitude  = 37.565350
wp2.longitude = 126.626778
wp2.altitude  = 100.0
wp2.yaw       = np.pi/180.0 * 210.0

wp3.coordinate_frame = wp0.coordinate_frame
wp3.type_mask = wp0.type_mask
wp3.latitude  = 37.564700
wp3.longitude = 126.627628
wp3.altitude  = 2.0
wp3.yaw       = np.pi/180.0 * 313.0

wp4.coordinate_frame = wp0.coordinate_frame
wp4.type_mask = wp0.type_mask
wp4.latitude  = 37.565111
wp4.longitude = 126.628503
wp4.altitude  = 2.0
wp4.yaw       = np.pi/180.0 * 28.0


wp.coordinate_frame = wp0.coordinate_frame
wp.type_mask = wp0.type_mask
wp.latitude = wp0.latitude
wp.longitude = wp0.longitude
wp.altitude = wp0.altitude

wp_target.coordinate_frame = wp2.coordinate_frame
wp_target.type_mask = wp2.type_mask
wp_target.latitude = wp2.latitude
wp_target.longitude = wp2.longitude
wp_target.altitude = wp2.altitude
wp_target.yaw = wp3.yaw

wp_avoidance.coordinate_frame = wp3.coordinate_frame
wp_avoidance.type_mask = wp3.type_mask
wp_avoidance.latitude = wp3.latitude
wp_avoidance.longitude = wp3.longitude
wp_avoidance.altitude = wp3.altitude
wp_avoidance.yaw = wp4.yaw

wp_helical.coordinate_frame = wp4.coordinate_frame
wp_helical.type_mask = wp4.type_mask
wp_helical.latitude = wp4.latitude
wp_helical.longitude = wp4.longitude
wp_helical.altitude = wp4.altitude
wp_helical.yaw = wp4.yaw

waypoint = Int32()
process = Int32()


def homeCb(msg):
    home.geo.latitude = msg.geo.latitude
    home.geo.longitude = msg.geo.longitude
    home.geo.altitude = msg.geo.altitude

    wp0.latitude = msg.geo.latitude
    wp0.longitude = msg.geo.longitude


def stateCb(msg):
    current_state.mode = msg.mode


def positionCb(msg):
    current_position.latitude = msg.latitude
    current_position.longitude = msg.longitude
    current_position.altitude = msg.altitude


def targetCb(msg):
    wp_target.latitude = msg.latitude
    wp_target.longitude = msg.longitude
    wp_target.altitude = msg.altitude
    wp_target.yaw = msg.yaw


def avoidanceCb(msg):
    wp_avoidance.latitude = msg.latitude
    wp_avoidance.longitude = msg.longitude
    wp_avoidance.altitude = msg.altitude
    wp_avoidance.yaw = msg.yaw


def helicalCb(msg):
    wp_helical.latitude = msg.latitude
    wp_helical.longitude = msg.longitude
    wp_helical.altitude = msg.altitude
    wp_helical.yaw = msg.yaw


def getDistance():
    """
    Call before changing altitude into AMSL
    """
    h = geopy.distance.vincenty((wp.latitude, wp.longitude), (current_position.latitude, current_position.longitude)).m
    v = (wp.altitude + home.geo.altitude) - current_position.altitude
    print("Distance to next waypoint: %.1f, %.1f" %(h, v))
    return h, v


if __name__ == "__main__":
    # Initialize node
    rospy.init_node('riseq_sacc_main')

    # Set loop rate
    rate = 50
    r = rospy.Rate(rate)

    # Subscriber
    rospy.Subscriber('/mavros/state', State, stateCb)
    rospy.Subscriber('/mavros/home_position/home', HomePosition, homeCb)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, positionCb)
    rospy.Subscriber('/setpoint_target', GlobalPositionTarget, targetCb)
    rospy.Subscriber('/setpoint_avoidance', GlobalPositionTarget, avoidanceCb)
    rospy.Subscriber('/riseq/sacc/setpoint_helix_global', GlobalPositionTarget, helicalCb)
    
    # Publisher
    position_publisher = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)
    waypoint_publisher = rospy.Publisher('/waypoint', Int32, queue_size=10)
    process_publisher = rospy.Publisher('/process', Int32, queue_size=10)

    # Service client
    set_home_client = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    landing_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

    rospy.sleep(1.0)


    # Set home position and update waypoints' altitude on ground
    while True:
        set_home = set_home_client(True, 0.0, 0.0, 0.0)
        r.sleep()
        if set_home.success is True:
            print(">>>>>Set home position<<<<<\n")
            break
    rospy.sleep(3.0)
    print(">>>>>Updated altitude<<<<\n")
    print(">>>>>Arm<<<<<\n")


    # Main loop publishing setpoint
    step = 0
    while not rospy.is_shutdown():
        # Takeoff
        if step == 0:
            wp.latitude = wp0.latitude
            wp.longitude = wp0.longitude
            wp.altitude = wp0.altitude
            wp.yaw = wp0.yaw
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
            waypoint.data = 1

        # wp0 -> wp1
        elif step == 1:
            wp.latitude = wp1.latitude
            wp.longitude = wp1.longitude
            wp.altitude = wp1.altitude
            wp.yaw = wp1.yaw
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
            waypoint.data = 1

        # wp1 -> wp2
        elif step == 2:
            wp.latitude = wp2.latitude
            wp.longitude = wp2.longitude
            wp.altitude = wp2.altitude
            wp.yaw = wp2.yaw
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
                process.data = 1
            waypoint.data = 2

        # wp2 -> wp3, Track target
        elif step == 3:
            process_publisher.publish(process)
            wp.latitude = wp_target.latitude
            wp.longitude = wp_target.longitude
            wp.altitude = wp_target.altitude
            wp.yaw = wp_target.yaw
            if (geopy.distance.vincenty((wp3.latitude, wp3.longitude), (current_position.latitude, current_position.longitude)).m <= 2.0) and (abs(wp3.altitude - (current_position.altitude - home.geo.altitude)) <= 2.0):
                step += 1
                process.data = 2
            waypoint.data = 3

        # Fix orientation
        elif step == 4:
            wp.latitude = wp_avoidance.latitude
            wp.longitude = wp_avoidance.longitude
            wp.altitude = wp_avoidance.altitude + home.geo.altitude
            wp.yaw = wp_avoidance.yaw
            for i in range(0, rate*3):
                wp.header.stamp = rospy.Time.now()
                position_publisher.publish(wp)
                r.sleep()
            step += 1
            wp.altitude = wp_avoidance.altitude
        # wp3 -> wp4, Obstacle avoidance
        elif step == 5:
            process_publisher.publish(process)
            wp.latitude = wp_avoidance.latitude
            wp.longitude = wp_avoidance.longitude
            wp.altitude = wp_avoidance.altitude
            wp.yaw = wp_avoidance.yaw
            if (geopy.distance.vincenty((wp4.latitude, wp4.longitude), (current_position.latitude, current_position.longitude)).m <= 2.0) and (abs(wp4.altitude - (current_position.altitude - home.geo.altitude)) <= 2.0):
                step += 1
                process.data = 3
            waypoint.data = 4

        # Wait YOLOv3 and depth estimator running
        elif step == 6:
            process_publisher.publish(process)
            wp.latitude = wp4.latitude
            wp.longitude = wp4.longitude
            wp.altitude = wp4.altitude + home.geo.altitude
            wp.yaw = wp4.yaw
            for i in range(0, rate*16):
                wp.header.stamp = rospy.Time.now()
                position_publisher.publish(wp)
                r.sleep()
            step += 1
        # Helical trajectory, Number segmentation
        elif step == 7:
            process_publisher.publish(process)
            wp.latitude = wp_helical.latitude
            wp.longitude = wp_helical.longitude
            wp.altitude = wp_helical.altitude + home.geo.altitude
            wp.yaw = wp_helical.yaw
            if (current_position.altitude >= (35.0 + home.geo.altitude)):
                step += 1
                process.data = 4
            waypoint.data = 0

        # Return home
        elif step == 8:
            wp.latitude = wp0.latitude
            wp.longitude = wp0.longitude
            wp.altitude = 35.0
            wp.yaw = wp0.yaw
            err_h, err_v = getDistance()
            if (err_h <= 2.0) and (abs(err_v) <= 2.0):
                step += 1
            waypoint.data = 0

        # Land
        elif step == 9:
            wp.latitude = wp0.latitude
            wp.longitude = wp0.longitude
            wp.altitude = 2.0
            wp.yaw = wp0.yaw
            err_h, err_v = getDistance()
            if (err_h <= 1.0) and (abs(err_v) <= 0.5):
                step += 1
            waypoint.data = 0
        elif step == 10:
            wp.latitude = wp0.latitude
            wp.longitude = wp0.longitude
            wp.altitude = 2.0
            wp.yaw = wp0.yaw
            err_h, err_v = getDistance()
            if (err_h <= 1.0) and (abs(err_v) <= 0.5) and (current_state.mode != "AUTO.LAND"):
                print(">>>>>Landing<<<<<\n")
                land_home = landing_client(0.0, 0.0, 0.0, 0.0, 0.0)
            waypoint.data = 0


        # Publish position and heading waypoint
        wp.header.stamp = rospy.Time.now()
        wp.altitude = wp.altitude + home.geo.altitude    # Change relative altitude into GPS altitude(AMSL)
        position_publisher.publish(wp)
        waypoint_publisher.publish(waypoint)
        
        r.sleep()

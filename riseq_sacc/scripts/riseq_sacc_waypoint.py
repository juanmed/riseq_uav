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
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget, HomePosition
from mavros_msgs.srv import CommandHome, CommandBool, SetMode, CommandTOL


class Waypoint():
    def stateCb(self, msg):
        self.current_state = msg

    def homeCb(self, msg):
        self.home_position = msg

    def positionCb(self, msg):
        self.current_position = msg


    def setWaypoints(self):

        # (library)
        #
        #  2 <- 1
        #  |    ^
        #  v    |
        #  3 -> 4
        self.wp1.header.stamp = rospy.Time.now()
        self.wp1.coordinate_frame = GlobalPositionTarget().FRAME_GLOBAL_TERRAIN_ALT
        self.wp1.type_mask = GlobalPositionTarget().IGNORE_VX + GlobalPositionTarget().IGNORE_VY + GlobalPositionTarget().IGNORE_VZ + GlobalPositionTarget().IGNORE_AFX + GlobalPositionTarget().IGNORE_AFY + GlobalPositionTarget().IGNORE_AFZ + GlobalPositionTarget().FORCE + GlobalPositionTarget().IGNORE_YAW_RATE
        self.wp1.latitude = 37.293336
        self.wp1.longitude = 126.974761
        self.wp1.altitude = 55.0
        self.wp1.yaw = 0.0

        self.wp2.header.stamp = rospy.Time.now()
        self.wp2.coordinate_frame = self.wp1.coordinate_frame
        self.wp2.type_mask = self.wp1.type_mask
        self.wp2.latitude = 37.293336
        self.wp2.longitude = 126.974561
        self.wp2.altitude = 55.0
        self.wp2.yaw = 0.0

        self.wp3.header.stamp = rospy.Time.now()
        self.wp3.coordinate_frame = self.wp1.coordinate_frame
        self.wp3.type_mask = self.wp1.type_mask
        self.wp3.latitude = 37.293136
        self.wp3.longitude = 126.974561
        self.wp3.altitude = 55.0
        self.wp3.yaw = 0.0

        self.wp4.header.stamp = rospy.Time.now()
        self.wp4.coordinate_frame = self.wp1.coordinate_frame
        self.wp4.type_mask = self.wp1.type_mask
        self.wp4.latitude = 37.293136
        self.wp4.longitude = 126.974761
        self.wp4.altitude = 55.0
        self.wp4.yaw = 0.0


    def getDistance(self, wp):
        """
        Calculate the distance between current position and waypoint in m.
        """
        cur = (self.current_position.latitude, self.current_position.longitude)
        if wp == 1:
            nxt = (self.wp1.latitude, self.wp1.longitude)
        elif wp == 2:
            nxt = (self.wp2.latitude, self.wp2.longitude)
        elif wp == 3:
            nxt = (self.wp3.latitude, self.wp3.longitude)
        elif wp == 4:
            nxt = (self.wp4.latitude, self.wp4.longitude)
        elif wp == 5:
            nxt = (self.wp1.latitude, self.wp1.longitude)

        if wp <= 5:
	    distance = geopy.distance.vincenty(cur, nxt).m
            print("Distance to next waypoint: %.2f" %distance)
            return distance
        else:
            print("There is no more waypont...")
            return 99.99


    def __init__(self):
        rospy.init_node('riseq_sacc_waypoint')
        self.rate = 50
        self.r = rospy.Rate(self.rate)

        self.wp = 1
        self.wp1 = GlobalPositionTarget()
        self.wp2 = GlobalPositionTarget()
        self.wp3 = GlobalPositionTarget()
        self.wp4 = GlobalPositionTarget()

        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber('/mavros/home_position/home', HomePosition, self.homeCb)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.positionCb)
        self.position_publisher = rospy.Publisher('/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=10)

        self.set_home_client = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.landing_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        while True:
            set_home = self.set_home_client(True, 0.0, 0.0, 0.0)
            self.r.sleep()
            if set_home.success is True:
                print(">>>>>Set home position<<<<<\n")
                break

        print("sending waypoint...")
        for i in range(self.rate * 10):
            self.setWaypoints()
            self.position_publisher.publish(self.wp1)
            self.r.sleep()


    def loop(self):
        if self.wp == 1:
            # Enter Offboard mode and then arm.
            if (self.current_state.mode == "AUTO.LOITER") and (self.current_state.mode != "OFFBOARD"):
                set_mode = self.set_mode_client(0, "OFFBOARD")
                print("entering offboard mode...")
                if set_mode.mode_sent is True:
                    print(">>>>>Offboard enabled<<<<<\n")
#            elif (not self.current_state.armed):
#                arming = self.arming_client(True)
#                print("arming...")
#                if arming.success is True:
#                    print(">>>>>Vehicle armed<<<<<\n")

        # Calculate the distance between waypoint and current position.
        if self.getDistance(self.wp) < 1.0:
            self.wp += 1

        # Set waypoint.
        self.setWaypoints()
        if self.wp == 1:
            self.position_publisher.publish(self.wp1)
        elif self.wp == 2:
            self.position_publisher.publish(self.wp2)
        elif self.wp == 3:
            self.position_publisher.publish(self.wp3)
        elif self.wp == 4:
            self.position_publisher.publish(self.wp4)
        elif self.wp == 5:
            self.position_publisher.publish(self.wp1)
        elif (self.wp > 5) and (self.current_state.mode != "AUTO.LAND"):
            print(">>>>>Landing<<<<<\n")
            self.landing_client(0.0, 0.0, 0.0, 0.0, 0.0)

        self.r.sleep()


if __name__ == "__main__":
    try:
        wp = Waypoint()
        while not rospy.is_shutdown():
            wp.loop()
    except rospy.ROSInterruptException:
        pass

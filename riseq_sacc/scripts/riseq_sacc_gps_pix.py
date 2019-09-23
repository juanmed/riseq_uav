#! /usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float64, Float64MultiArray, Int32
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix

current_state = State()
waypoint = Int32()
position = NavSatFix()

def stateCb(msg):
    current_state.mode = msg.mode

def waypointCb(msg):
    waypoint.data = msg.data

def globalCb(msg):
    position.latitude = msg.latitude
    position.longitude = msg.longitude
    position.altitude = msg.altitude

def waypointCb(msg):
    waypoint.data = msg.data

if __name__ == '__main__':
    f = open("/home/nvidia/Desktop/gps.asc",'w')

    rospy.init_node('gps_serial_reader')

    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 9600)


    rospy.Subscriber('/mavros/state', State, stateCb)
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, globalCb)
    rospy.Subscriber('/waypoint', Int32, waypointCb)


    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)


        gps_sat = []
        gps_data_string = ''

        while not rospy.is_shutdown():
            if (current_state.mode == "OFFBOARD"):
                gps_mode = 1
            elif (current_state.mode == "AUTO.LAND"):
                gps_mode = 1
            else :
                gps_mode =0

            data = GPS.readline().strip().split(',')

            if data[0] == '$GPGSA':
                if data[3] != '':
                    gps_sat = data[3:15]

                gps_data_string = str(gps_mode) + '	' + str(waypoint.data) + '	' + str(format(position.latitude,"10.6f"))  + '	' + str(format(position.longitude, "10.6f")) + '	' + str(format(position.altitude, "5.1f")) + '	' + ', '.join(gps_sat) + '\n'


                print(gps_data_string)
                f.write(gps_data_string)
                gps_sat = []

    except rospy.ROSInterruptException:
        GPS.close() #Close GPS serial portission.

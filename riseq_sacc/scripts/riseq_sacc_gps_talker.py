#! /usr/bin/env python

import serial
import rospy
from std_msgs.msg import Float32MultiArray

if __name__ == '__main__':
    f = open("/home/nvidia/Desktop/gps.asc",'w')

    rospy.init_node('gps_serial_reader')

    gps_pub = rospy.Publisher("gps_data", Float32MultiArray, queue_size=1)


    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud',9600)

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)

        gps_data = []
        gps_sat = []
        gps_data_string = ''
        rout = 1

        while not rospy.is_shutdown():
            data = GPS.readline().strip().split(',')

            if data[0] == '$GPGGA':
                if data[2] != '':
                    gps_data.append('1')
                    gps_data.append(str(rout))
                    gps_data.append(data[1])
                    gps_data.append(str(format(int(data[2][0:2])+float(data[2][2:])/60,"10.6f")))
                    gps_data.append(str(format(int(data[4][0:3])+float(data[4][3:])/60,"10.6f")))
                    gps_data.append(data[9])

                    gps = Float32MultiArray()
                    gps.data = list(map(float,gps_data))

                    gps_pub.publish(gps)

            if data[0] == '$GPGSA':
                if data[3] != '':
                    gps_sat = data[3:15]

                gps_data_string = '  '.join(gps_data) + '  ' + ' '.join(gps_sat) + '\n'
                gps_data = []
                gps_sat = []     
           
                print(gps_data_string)
                f.write(gps_data_string)

    except rospy.ROSInterruptException:
        GPS.close() #Close GPS serial portission.

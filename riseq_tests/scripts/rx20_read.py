# Test to obtain serial data from RX-20 Sensor
# Hopefully the manual can be found here:
# http://www.cpny.co.kr/_public/_uploadFiles/neo_pd_img/l/CUKF1FXY2P1P1DFNSSFT.pdf


import serial
import numpy
import time
import re

import matplotlib.pyplot as plt

def init_serial(port, baudrate):
    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )   

    if(ser.isOpen()):

        # Read program version
        ser.write('RDVR\r')
        time.sleep(0.1)

        version = ''
        while ser.inWaiting() > 0:
            version += ser.read(1)

        # Read allowable overload
        ser.write('RDMDL\r')
        time.sleep(0.1)

        overload = ''
        while ser.inWaiting() > 0:
            overload += ser.read(1)

        # Read current value
        ser.write('RDF0\r')
        time.sleep(0.1)

        val = ''
        while ser.inWaiting() > 0:
            val += ser.read(1)


        print('AIKOH RX-2 Force Sensor Interface program \n'+
              'Serial Port: {} \n'.format(ser.name)+
              'Baudrate: {}\n'.format(baudrate)+
              'Program version: {}\n'.format(version)+
              'Allowable Overload {}\n'.format(overload)+
              'Current read {}\n'.format(val))
    else:
        print('Problem while opening {}'.format(port))
        ser = None

    return ser



def serial_data(port, baudrate):
    ser = serial.Serial(port, baudrate)

    while True:
        yield ser.readline()

    ser.close()

# init gauge
gauge = init_serial('/dev/ttyUSB0', 38400)

# enable continuous data 
#print(gauge.write('RDF1R1\r'))

# read data
bytes = 1000
data = list()

request = True                  # flag to request data
while len(data) < bytes:

    # request data
    if request:
        gauge.write('RDF0\r')
        request = False

    # check if data is available
    if(gauge.inWaiting()):
        val = ''
        while gauge.inWaiting() > 0 : 
            val += gauge.read(1)
        data.append(val)
        request = True


    
data = [float(re.findall(r"[-+]?\d*\.\d+|\d+", value)[0]) for value in data]
#print (data)
#data = [float(value) for value in data]

fig0 = plt.figure(figsize=(20,10))
ax0 = fig0.add_subplot(111)
ax0.plot(data, color = 'r', linestyle = '-', label = 'Force {N}')
ax0.legend(loc='upper right', shadow=True, fontsize='medium')
ax0.set_title('Force Reading using AIKOH RX-2')
ax0.set_ylabel('Force {N}')
ax0.set_xlabel('n')

plt.show()

gauge.close()
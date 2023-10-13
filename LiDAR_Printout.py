#!/usr/bin/env python3

import serial
import numpy

#use to transfer file to RPi
# $ sftp johndanger@raspberrypi.local
# $ put /Users/jdanger/Desktop/Mobile_Robotics/LiDAR_Printout.py /home/johndanger/catkin_ws/src/hmwk1/scripts/LiDAR_Printout.py
# $ put C:/Users/claassen/Desktop/Mobile_Robotics/LiDAR_Printout.py /home/johndanger/catkin_ws/src/hmwk1/scripts/LiDAR_Printout.py


port_by_connection_type = {
    'Mac' : '/dev/tty.usbserial-0001',
    'RPi' : '/dev/ttyUSB0',
    'Windows' : 'COM4',
}


with serial.Serial(port_by_connection_type['RPi']) as conn:

    conn.baudrate = 230400
    conn.bytesize = serial.EIGHTBITS
    conn.stopbits = serial.STOPBITS_ONE
    conn.parity = serial.PARITY_NONE
    conn.timeout = 5

    timeout_counter = 0
    while timeout_counter < 10:

        byteString = conn.read_until(b'\x54') #header will be at end
        packet = list(byteString)
        if len(packet) != 47 or packet[0] != 0x2C:  #packet info starts list
            continue
            timeout_counter += 1

        print(packet)
        timeout_counter += 1

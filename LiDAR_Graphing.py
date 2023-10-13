#!/usr/bin/env python3

import math
import serial
from matplotlib import pyplot as plt  # I think this caused Gtk warning when ran on RPi via ssh
from objs_LiDAR import D300_LiDAR

#use to transfer file to RPi
# $ sftp johndanger@raspberrypi.local
# $ put /Users/jdanger/Desktop/Mobile_Robotics/LiDAR_Graphing.py /home/johndanger/catkin_ws/src/hmwk1/scripts/LiDAR_Processing.py
# $ put C:/Users/claassen/Desktop/Mobile_Robotics/LiDAR_Graphing.py /home/johndanger/catkin_ws/src/hmwk1/scripts/LiDAR_Processing.py



# try UTM for running ubuntu on osx


##
lidarObj = D300_LiDAR('RPi')
with lidarObj as lidarConn:

    packetDataByScan = {}
    packetInfoByScan = {}
    while lidarObj.scan_counter < 1000 and lidarObj.timeout_counter < 10000:

        lidarObj.read_packet()
        if not lidarObj.buffer:
            continue

        info, data = lidarObj.process_packet()
        packetInfoByScan[lidarObj.scan_counter] = info
        packetDataByScan[lidarObj.scan_counter] = data

        lidarObj.scan_counter += 1

print(packetInfoByScan)




#points will be plotted in cartesian space
allX, allY = [], []
for scanNum, scanInfo in packetInfoByScan.items():

    if scanNum == 0:  #first scan cannot have angle traveled calculated for it
        continue

    #unpack scan info
    speed, startAngle, endAngle, timestamp, crc = scanInfo

    #compare the angle travelled as calculated by the rotational speed and the measured angles
    elapsedTime = timestamp - packetInfoByScan[scanNum-1][3]  #timestamp of previous scan
    # totalAngle_speed = elapsedTime * speed
    totalAngle_angle = endAngle - startAngle
    # print(f'Angle diff: {round(totalAngle_speed - totalAngle_angle, 2)} degrees')

    #calculate angle travelled using total angle from two different methods
    # angleBetweenPoints_speed = totalAngle_speed / (len(packetDataByScan[scanNum]) - 1)
    angleBetweenPoints_angle = totalAngle_angle / (len(packetDataByScan[scanNum]) - 1)
    for pointNum, (distance, intensity) in enumerate(packetDataByScan[scanNum]):
        pointAngle = startAngle + angleBetweenPoints_angle*pointNum  #has shifted section around 0 degrees
        # pointAngle = startAngle + angleBetweenPoints_speed*pointNum  #similar to above but doesn't have shifted section
        #convert angle and distance into cartesian coordinates
        allX.append(distance * math.cos(math.deg2rad(pointAngle)))
        allY.append(distance * math.sin(math.deg2rad(pointAngle)))


plt.scatter(allX, allY, s=.1)
plt.show()


##








#!/usr/bin/env python3

import math
import serial
from matplotlib import pyplot as plt  # I think this caused Gtk warning when ran on RPi via ssh
from objs_LiDAR import D300_LiDAR
from matplotlib.cbook import get_sample_data

#use to transfer file to RPi
# $ sftp johndanger@raspberrypi.local
# $ put /Users/jdanger/Desktop/Mobile_Robotics/LiDAR_Graphing.py /home/johndanger/catkin_ws/src/hmwk1/scripts/LiDAR_Processing.py
# $ put C:/Users/claassen/Desktop/Mobile_Robotics/LiDAR_Graphing.py /home/johndanger/catkin_ws/src/hmwk1/scripts/LiDAR_Processing.py



# try UTM for running ubuntu on osx


##
lidarObj = D300_LiDAR('Mac')
with lidarObj as lidarConn:

    packetDataByScan = {}
    packetInfoByScan = {}
    while lidarObj.scan_counter < 500 and lidarObj.timeout_counter < 10000:

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
        if 30 < distance < 500:

            pointAngle = startAngle + angleBetweenPoints_angle*pointNum  #has shifted section around 0 degrees
            # pointAngle = startAngle + angleBetweenPoints_speed*pointNum  #similar to above but doesn't have shifted section
            #convert angle and distance into cartesian coordinates
            x = distance * math.cos(-math.radians(pointAngle))
            y = distance * math.sin(-math.radians(pointAngle))

            allX.append(x)
            allY.append(y)

##

fig, ax = plt.subplots(1)
ax.set_title('LiDAR Scan of Lab')
ax.set_xlabel('Distance (mm)')
ax.set_ylabel('Distance (mm)')
ax.scatter(allX, allY, s=.1)
ax.scatter(0, 0, s=50)
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)

im = plt.imread(get_sample_data('/Users/jdanger/Desktop/Mobile_Robotics/desktop_LiDAR.jpg'))

# Place the image in the upper-right corner of the figure
#--------------------------------------------------------
# We're specifying the position and size in _figure_ coordinates, so the image
# will shrink/grow as the figure is resized. Remove "zorder=-1" to place the
# image in front of the axes.
# left, bottom, width, height
newax = fig.add_axes([0.2, 0.33, 0.55, 0.55], anchor='NE')#, zorder=-1)
newax.imshow(im, alpha=.3)
newax.axis('off')

fig.show()

##








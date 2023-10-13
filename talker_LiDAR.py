#!/usr/bin/env python3

import rospy
import rosbag
from std_msgs.msg import String

import math
from objs_LiDAR import D300_LiDAR

#use to transfer file to RPi
# $ sftp johndanger@raspberrypi.local
# $ put /Users/jdanger/Desktop/Mobile_Robotics/talker_LiDAR.py /home/johndanger/catkin_ws/src/hmwk1/scripts/talker_LiDAR.py
# $ put C:/Users/claassen/Desktop/Mobile_Robotics/talker_LiDAR.py /home/johndanger/catkin_ws/src/hmwk1/scripts/talker_LiDAR.py


# try UTM for running ubuntu on osx


def talker_LiDAR_Cartesian():

    #Node will publish to "LiDAR_readings" topic, with String message type
    pub = rospy.Publisher('LiDAR_readings', String, queue_size=10)

    bag = rosbag.Bag('Minute_LiDAR_bag', 'w')

    #name Node "LiDAR_talker", anonymous adds numbers to name to make it unique
    rospy.init_node('LiDAR_talker', anonymous=True)
    #runs process 10x/second (if process is short enough for this to be possible)
    rate = rospy.Rate(10)  #hz


    lidarObj = D300_LiDAR('RPi')
    with lidarObj:

        #pubish string if process is still running (no ctrl-C etc.)
        while not rospy.is_shutdown():

            lidarObj.read_packet()
            if not lidarObj.buffer:
                continue

            #parse bytestring to get packet info and data
            info, data = lidarObj.process_packet()
            speed, startAngle, endAngle, timestamp, crc = info

            #points will be plotted in cartesian space
            #calculate angle travelled using total angle from two different methods
            angleBetweenPoints = (endAngle - startAngle) / (len(data) - 1)
            for pointNum, (distance, intensity) in enumerate(data):

                #convert angle and distance into cartesian coordinates
                pointAngle = startAngle + angleBetweenPoints * pointNum
                x = distance * math.cos(math.radians(pointAngle))
                y = distance * math.sin(math.radians(pointAngle))
                dataStr = f'Point at ({x}, {y}), intensity {intensity}'

                #written to screen, Node log, and rosout (see with rqt_console)
                rospy.loginfo(dataStr)
                #publishes to "LiDAR_readings" topic
                pub.publish(dataStr)
                #stores in bag
                bag.write(dataStr)

            rate.sleep()  #sleeps for remainder of 1/10th of a second to heep 10 hz process rate





if __name__ == '__main__':

    try:  #running publisher but stop for ctrl-C or other node shutdown
        talker_LiDAR_Cartesian()
    except rospy.ROSInterruptException:
        pass




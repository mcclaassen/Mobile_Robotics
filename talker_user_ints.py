#!/usr/bin/env python3
# the above runs this file as a python3 script

#use to transfer file to RPi
# $ sftp johndanger@raspberrypi.local
# $ put /Users/jdanger/Desktop/Mobile_Robotics/talker_user_ints.py /home/johndanger/catkin_ws/src/hmwk1/scripts/talker_user_ints.py

import rospy
from std_msgs.msg import String

def talker():

    #Node will publish to "user_ints" topic, with String message type
    pub = rospy.Publisher('user_ints', String, queue_size=10)
    #name Node "talker", anonymous adds numbers to name to make it unique
    rospy.init_node('talker', anonymous=True)
    #runs process 10x/second (if process is short enough for this to be possible)
    rate = rospy.Rate(10)  #hz

    #pubish string if process is still running (no ctrl-C etc.)
    while not rospy.is_shutdown():
        user_int = input('Enter integer:\t').strip()
        rospy.loginfo(user_int)  #written to screen, Node log, and rosout (see with rqt_console)
        pub.publish(user_int)  #publishes to "chatter" topic
        rate.sleep()  #sleeps for remainder of 1/10th of a second to heep 10 hz process rate


if __name__ == '__main__':

    try:  #running publisher but stop for ctrl-C or other node shutdown
        talker()
    except rospy.ROSInterruptException:
        pass



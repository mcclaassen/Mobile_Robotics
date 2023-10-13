#!/usr/bin/env python3

# use to transfer file to
# $ sftp johndanger@raspberrypi.local
# $ put /Users/jdanger/Desktop/Mobile_Robotics/listener_product_ints.py /home/johndanger/catkin_ws/src/hmwk1/scripts/listener_product_ints.py


import rospy
from std_msgs.msg import String


def callback(user_supplied_int, all_ints):
    """
        Show what publisher was heard & print product of all input integers

        all_ints:   list of all integers that have been input by user
    """

    #write published int to std out, log, & rosout
    rospy.loginfo(rospy.get_caller_id() + f'I heard {user_supplied_int.data}')
    #store newly published int
    all_ints.append(user_supplied_int.data)

    #print product of all published ints
    product = 1
    for userInt in all_ints:
        product *= int(userInt)
    print(f'Total product = {product}')


def listener():
    """
        Listen for published ints
    """

    allInts = []  #initialize int storage
    rospy.init_node('listener', anonymous=True)  # name node (with unique #s at end with anonymous arg)

    #subscribe node to "user_ints" topic and process each published message/int
    rospy.Subscriber('user_ints', String, callback, allInts)  #args: topic name, dtype of message, process fn, fn args
    rospy.spin()  # keeps script running until node is shutdown


if __name__ == '__main__':

    listener()

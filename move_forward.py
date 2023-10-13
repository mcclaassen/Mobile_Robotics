#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_forward():
    rospy.init_node('move_forward')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size= 10)

    try: 
        duration = int(input("Enter the time duration= "))
    except ValueError:
        print("Please input an integer.")
        return
    
    vel_msg = Twist()
    vel_msg.linear.x = 1.0

    start_time = rospy.Time.now()
    

    while not rospy.is_shutdown() and ((rospy.Time.now() - start_time).to_sec() < duration):
        pub.publish(vel_msg)
        

    vel_msg.linear.x = 0
    pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass 

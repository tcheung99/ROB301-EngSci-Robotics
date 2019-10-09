#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String



def publisher_node():
    print('TODO: initialize the publisher node here, \
            and publish wheel command to the cmd_vel topic')
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    twist=Twist()
    init_t = rospy.get_time()
    rate = rospy.Rate(10)
    while rospy.get_time()-init_t < (17.929):
	twist.linear.x = 0.1
	cmd_pub.publish(twist)
	rospy.loginfo(twist.linear.x)
        rate.sleep()
    init_t = rospy.get_time()
    while rospy.get_time()-init_t < (15):
	twist.linear.x = 0.046
	twist.angular.z = 0.157079632
	cmd_pub.publish(twist)
	rospy.loginfo(twist.linear.x)
        rate.sleep()
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_pub.publish(twist)
    pass


def main():
    try:
        rospy.init_node('motor')
        publisher_node()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == '__main__':
    main()
